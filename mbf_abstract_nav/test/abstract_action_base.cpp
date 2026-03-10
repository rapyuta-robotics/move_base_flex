#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <future>

// dummy message
#include <mbf_msgs/GetPathAction.h>
#include <mbf_utility/robot_information.h>

#include <mbf_abstract_nav/abstract_action_base.hpp>
#include <mbf_abstract_nav/abstract_execution_base.h>

using namespace mbf_abstract_nav;

// mocked version of an execution
struct MockedExecution : public AbstractExecutionBase {
  typedef boost::shared_ptr<MockedExecution> Ptr;

  MockedExecution(const mbf_utility::RobotInformation& ri) : AbstractExecutionBase("mocked_execution", ri) {}

  MOCK_METHOD0(cancel, bool());

protected:
  MOCK_METHOD0(run, void());
};

using testing::Test;

// fixture with access to the AbstractActionBase's internals
struct AbstractActionBaseFixture
    : public AbstractActionBase<mbf_msgs::GetPathAction, MockedExecution>,
      public Test {
  // required members for the c'tor
  TF tf_;
  std::string test_name;
  mbf_utility::RobotInformation ri;

  AbstractActionBaseFixture()
      : test_name("action_base"),
        ri(tf_, "global_frame", "local_frame", ros::Duration()),
        AbstractActionBase(test_name, ri)
  {
  }

  void runImpl(GoalHandle &goal_handle, MockedExecution &execution) {}
};

struct CancelExecution : public AbstractExecutionBase {
  typedef boost::shared_ptr<CancelExecution> Ptr;

  struct RunningCancelAction {
    boost::mutex mtx;
    boost::condition_variable condition_variable;
    bool execution_completed{false};

    RunningCancelAction() : execution_completed(false) {}
  };

  CancelExecution(const mbf_utility::RobotInformation& ri, const boost::shared_ptr<RunningCancelAction>& cancel_action)
    : AbstractExecutionBase("cancel_execution", ri), cancel_action(cancel_action) {}

  bool cancel() override
  {
    return true;
  }

protected:
  void run() override
  {
    boost::unique_lock<boost::mutex> lock(cancel_action->mtx);
    while (!cancel_action->execution_completed)
    {
      cancel_action->condition_variable.wait(lock);
    }
  }

private:
  boost::shared_ptr<RunningCancelAction> cancel_action;
};

struct NonBlockingCancelFixture : public Test
{
  typedef actionlib::ServerGoalHandle<mbf_msgs::GetPathAction> GoalHandle;

  struct TestAction : public AbstractActionBase<mbf_msgs::GetPathAction, CancelExecution>
  {
    TestAction(const std::string &name,
               const mbf_utility::RobotInformation &robot_info,
               const boost::shared_ptr<CancelExecution::RunningCancelAction>& cancel_action)
      : AbstractActionBase(name, robot_info), cancel_action(cancel_action) {}

    void runImpl(GoalHandle &goal_handle, CancelExecution &execution) override
    {
      (void)goal_handle;
      execution.start();
    }

    boost::shared_ptr<CancelExecution::RunningCancelAction> cancel_action;
  };

  struct MockedActionServer : public actionlib::ActionServerBase<mbf_msgs::GetPathAction>
  {
    typedef actionlib::ServerGoalHandle<mbf_msgs::GetPathAction> GoalHandle;

    MockedActionServer(boost::function<void(GoalHandle)> goal_cb, boost::function<void(GoalHandle)> cancel_cb)
      : ActionServerBase(goal_cb, cancel_cb, true)
    {
    }

    void initialize() override {}
    void publishResult(const actionlib_msgs::GoalStatus&, const Result&) override {}
    void publishFeedback(const actionlib_msgs::GoalStatus&, const Feedback&) override {}
    void publishStatus() override {}
  };

  TF tf_;
  std::string test_name;
  mbf_utility::RobotInformation ri;
  boost::shared_ptr<CancelExecution::RunningCancelAction> cancel_action;
  MockedActionServer action_server;
  TestAction action;
  mbf_msgs::GetPathActionGoalPtr first_goal;
  mbf_msgs::GetPathActionGoalPtr second_goal;

  NonBlockingCancelFixture()
    : test_name("action_base_non_blocking_start")
    , ri(tf_, "global_frame", "local_frame", ros::Duration())
    , cancel_action(boost::make_shared<CancelExecution::RunningCancelAction>())
    , action_server(boost::bind(&NonBlockingCancelFixture::callAction, this, _1),
                    boost::bind(&NonBlockingCancelFixture::cancelAction, this, _1))
    , action(test_name, ri, cancel_action)
    , first_goal(new mbf_msgs::GetPathActionGoal())
    , second_goal(new mbf_msgs::GetPathActionGoal())
  {
    first_goal->goal.concurrency_slot = 1;
    second_goal->goal.concurrency_slot = 1;
  }

  void callAction(GoalHandle goal_handle)
  {
    action.start(goal_handle, boost::make_shared<CancelExecution>(ri, cancel_action));
  }

  void cancelAction(GoalHandle goal_handle)
  {
    action.cancel(goal_handle);
  }

  void FinishCancelExecution()
  {
    boost::lock_guard<boost::mutex> lock(cancel_action->mtx);
    cancel_action->execution_completed = true;
    cancel_action->condition_variable.notify_all();
  }

  ~NonBlockingCancelFixture() override
  {
    FinishCancelExecution();
  }
};

TEST_F(AbstractActionBaseFixture, thread_stop)
{
  unsigned char slot = 1;
  concurrency_slots_[slot].execution.reset(new MockedExecution(AbstractActionBaseFixture::ri));
  concurrency_slots_[slot].thread_ptr =
      threads_.create_thread(boost::bind(&AbstractActionBaseFixture::run, this,
                                         boost::ref(concurrency_slots_[slot])));
}

using testing::Return;

TEST_F(AbstractActionBaseFixture, cancelAll)
{
  // spawn a bunch of threads
  for (unsigned char slot = 0; slot != 10; ++slot) {
    concurrency_slots_[slot].execution.reset(new MockedExecution(AbstractActionBaseFixture::ri));
    // set the expectation
    EXPECT_CALL(*concurrency_slots_[slot].execution, cancel())
        .WillRepeatedly(Return(true));

    // set the in_use flag --> this should turn to false
    concurrency_slots_[slot].in_use = true;
    concurrency_slots_[slot].thread_ptr = threads_.create_thread(
        boost::bind(&AbstractActionBaseFixture::run, this,
                    boost::ref(concurrency_slots_[slot])));
  }

  // cancel all of slots
  cancelAll();

  // check the result
  for (ConcurrencyMap::iterator slot = concurrency_slots_.begin();
       slot != concurrency_slots_.end(); ++slot)
    ASSERT_FALSE(slot->second.in_use);
}

TEST_F(NonBlockingCancelFixture, startDoesNotBlockOnCancelJoin)
{
  // first goal occupies slot 1 and starts a long-running execution
  action_server.goalCallback(first_goal);

  // second goal on same slot used to block in start() while joining canceled execution
  std::future<void> second_start = std::async(std::launch::async, [this]() {
    action_server.goalCallback(second_goal);
  });

  EXPECT_EQ(std::future_status::ready, second_start.wait_for(std::chrono::milliseconds(100)));
  FinishCancelExecution();
  EXPECT_EQ(std::future_status::ready, second_start.wait_for(std::chrono::seconds(1)));
}

int main(int argc, char **argv)
{
  // we need this only for kinetic and lunar distros
  ros::init(argc, argv, "abstract_action_base");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}