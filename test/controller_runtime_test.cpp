#include <gtest/gtest.h>

#include <chrono>

#include <ros/ros.h>

#include "controller_runtime/io/input_store.h"
#include "controller_runtime/io/topic_buffer.h"
#include "controller_runtime/scheduler/module_scheduler.h"
#include "controller_runtime/time/loop_controller.h"

namespace controller_runtime {
namespace {

TickContext makeCtx(double wall_s, double ros_s, bool ros_valid = true) {
    TickContext ctx;
    ctx.wall_now = ros::WallTime(wall_s);
    ctx.ros_now = ros::Time(ros_s);
    ctx.ros_time_valid = ros_valid;
    return ctx;
}

TEST(LoopControllerTest, BuildsTickContextWithWallAndRosTime) {
    LoopControllerOptions options;
    options.frequency_hz = 500.0;
    options.ros_jump_forward_threshold_s = 0.5;
    LoopController loop(options);

    ros::Time::setNow(ros::Time(1.0));
    const TickContext first = loop.makeTickContext();
    EXPECT_EQ(first.seq, 0u);
    EXPECT_TRUE(first.ros_time_valid);

    ros::Time::setNow(ros::Time(1.01));
    const TickContext second = loop.makeTickContext();
    EXPECT_EQ(second.seq, 1u);
    EXPECT_GT(second.wall_dt, 0.0);
    EXPECT_NEAR(second.ros_dt, 0.01, 1e-9);

    ros::Time::setNow(ros::Time(0.5));
    const TickContext back = loop.makeTickContext();
    EXPECT_TRUE(back.ros_time_jumped_back);

    ros::Time::setNow(ros::Time(2.0));
    const TickContext forward = loop.makeTickContext();
    EXPECT_TRUE(forward.ros_time_jumped_forward);
}

TEST(LoopControllerTest, WallLoopKeepsRunningWhenRosTimeInvalid) {
    LoopControllerOptions options;
    options.frequency_hz = 500.0;
    LoopController loop(options);

    ros::Time::setNow(ros::Time(0.0));

    int ticks = 0;
    int invalid_ros_ticks = 0;
    const auto start = std::chrono::steady_clock::now();
    loop.run([&](const TickContext& ctx) {
        ++ticks;
        if (!ctx.ros_time_valid) {
            ++invalid_ros_ticks;
        }
        if (ticks >= 30) {
            loop.requestStop();
        }
    });
    const auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

    EXPECT_EQ(ticks, 30);
    EXPECT_EQ(invalid_ros_ticks, 30);
    EXPECT_GT(elapsed, 0.03);
    EXPECT_LT(elapsed, 0.20);
}

TEST(TaskGateTest, HandlesPeriodDirtyMinPeriodAndClockDomain) {
    TaskSpec spec;
    spec.name = "health";
    spec.clock_domain = ClockDomain::Ros;
    spec.period_s = 0.1;
    spec.min_period_s = 0.02;
    spec.dirty_dependencies = {"pose"};
    TaskGate gate(spec);

    TickContext ctx = makeCtx(0.0, 0.0, false);
    EXPECT_FALSE(gate.evaluate(ctx, DirtySet{}).due);
    EXPECT_EQ(gate.evaluate(ctx, DirtySet{}).skip_reason, SkipReason::RosClockInvalid);

    ctx = makeCtx(0.0, 1.0);
    EXPECT_TRUE(gate.evaluate(ctx, DirtySet{}).due);
    gate.markRun(ctx);

    EXPECT_FALSE(gate.evaluate(makeCtx(0.01, 1.01), DirtySet{"pose"}).due);
    const TaskDecision dirty = gate.evaluate(makeCtx(0.03, 1.03), DirtySet{"pose"});
    EXPECT_TRUE(dirty.due);
    EXPECT_EQ(dirty.due_reason, DueReason::Dirty);
    gate.markRun(makeCtx(0.03, 1.03));

    const TaskDecision period = gate.evaluate(makeCtx(0.14, 1.14), DirtySet{});
    EXPECT_TRUE(period.due);
    EXPECT_EQ(period.due_reason, DueReason::Period);

    TaskSpec wall_spec;
    wall_spec.name = "diagnostics";
    wall_spec.clock_domain = ClockDomain::Wall;
    wall_spec.period_s = 0.1;
    TaskGate wall_gate(wall_spec);
    EXPECT_TRUE(wall_gate.evaluate(makeCtx(5.0, 0.0, false), DirtySet{}).due);
}

TEST(TopicBufferTest, TracksGenerationAndTickDirtySeparately) {
    TopicBuffer<int> buffer;
    EXPECT_FALSE(buffer.valid());
    EXPECT_EQ(buffer.generation(), 0u);

    const int msg = 42;
    buffer.update(msg, ros::Time(3.0), ros::WallTime(4.0));
    EXPECT_TRUE(buffer.valid());
    EXPECT_EQ(buffer.generation(), 1u);
    EXPECT_TRUE(buffer.dirtyThisTick());
    EXPECT_TRUE(buffer.hasNewSince(0));
    EXPECT_FALSE(buffer.hasNewSince(1));

    buffer.clearTickDirty();
    EXPECT_FALSE(buffer.dirtyThisTick());
    EXPECT_EQ(buffer.generation(), 1u);
}

TEST(InputStoreTest, BuildsGenerationSnapshotAndClearsTickDirty) {
    InputStore<int, int, std::string> store;
    store.updatePose(1, ros::Time(1.0), ros::WallTime(10.0));
    store.updateCommand("track", ros::Time(1.1), ros::WallTime(10.1));

    const InputSnapshot first = store.snapshot(makeCtx(10.2, 1.2));
    EXPECT_EQ(first.pose_generation, 1u);
    EXPECT_EQ(first.twist_generation, 0u);
    EXPECT_EQ(first.command_generation, 1u);
    EXPECT_NE(first.dirty.find("pose"), first.dirty.end());
    EXPECT_NE(first.dirty.find("command"), first.dirty.end());
    EXPECT_EQ(first.dirty.find("twist"), first.dirty.end());

    store.endTick();
    const InputSnapshot second = store.snapshot(makeCtx(10.3, 1.3));
    EXPECT_TRUE(second.dirty.empty());
    EXPECT_EQ(second.pose_generation, 1u);
    EXPECT_EQ(second.command_generation, 1u);
}

TEST(ModuleSchedulerTest, RunsMultiplePeriodsAndDirtyDependencies) {
    ModuleScheduler scheduler;
    int fast_runs = 0;
    int slow_runs = 0;
    int pose_dirty_runs = 0;

    TaskSpec fast;
    fast.name = "fast";
    fast.clock_domain = ClockDomain::Ros;
    fast.period_s = 0.02;
    fast.run_on_dirty = false;
    scheduler.addTask(fast, [&](const TickContext&, const DirtySet&) { ++fast_runs; });

    TaskSpec slow;
    slow.name = "slow";
    slow.clock_domain = ClockDomain::Ros;
    slow.period_s = 0.1;
    slow.run_on_dirty = false;
    scheduler.addTask(slow, [&](const TickContext&, const DirtySet&) { ++slow_runs; });

    TaskSpec pose_dirty;
    pose_dirty.name = "pose_dirty";
    pose_dirty.clock_domain = ClockDomain::Ros;
    pose_dirty.period_s = 100.0;
    pose_dirty.min_period_s = 0.0;
    pose_dirty.dirty_dependencies = {"pose"};
    scheduler.addTask(pose_dirty, [&](const TickContext&, const DirtySet&) { ++pose_dirty_runs; });

    for (int i = 0; i < 50; ++i) {
        DirtySet dirty;
        if (i == 7 || i == 11) {
            dirty.insert("pose");
        } else if (i == 20) {
            dirty.insert("twist");
        }
        scheduler.run(makeCtx(0.01 * i, 1.0 + 0.01 * i), dirty);
    }

    EXPECT_EQ(fast_runs, 25);
    EXPECT_EQ(slow_runs, 5);
    EXPECT_EQ(pose_dirty_runs, 3);  // first run plus two pose dirty triggers

    const auto stats = scheduler.stats();
    ASSERT_EQ(stats.size(), 3u);
    EXPECT_EQ(stats[0].run_count, static_cast<uint64_t>(fast_runs));
    EXPECT_GT(stats[0].skip_count, 0u);
    EXPECT_GE(stats[0].max_compute_ms, 0.0);
}

TEST(ModuleSchedulerTest, RespectsEnabledStates) {
    ModuleScheduler scheduler;
    int tracking_runs = 0;

    TaskSpec tracking_only;
    tracking_only.name = "tracking_only";
    tracking_only.clock_domain = ClockDomain::Wall;
    tracking_only.period_s = 0.0;
    tracking_only.enabled_states = {"TRACKING"};
    scheduler.addTask(tracking_only, [&](const TickContext&, const DirtySet&) { ++tracking_runs; });

    scheduler.run(makeCtx(1.0, 1.0), DirtySet{}, "READY");
    EXPECT_EQ(tracking_runs, 0);
    auto stats = scheduler.stats();
    ASSERT_EQ(stats.size(), 1u);
    EXPECT_EQ(stats[0].last_skip_reason, SkipReason::StateDisabled);

    scheduler.run(makeCtx(2.0, 2.0), DirtySet{}, "TRACKING");
    EXPECT_EQ(tracking_runs, 1);
    stats = scheduler.stats();
    EXPECT_EQ(stats[0].last_skip_reason, SkipReason::None);
}

}  // namespace
}  // namespace controller_runtime

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_runtime_test");
    ros::Time::init();
    ros::Time::setNow(ros::Time(0.0));
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
