import rospy
import actionlib

from usecase_controller_msgs.msg import PerformUseCaseAction
from usecase_controller_msgs.msg import PerformUseCaseGoal
from usecase_controller_msgs.msg import PerformUseCaseResult
from usecase_controller_msgs.msg import PerformUseCaseFeedback


class usecase1_driver_client_class(object):



    def __init__(self, name):
        self._action_name = name

        rospy.loginfo('%s: usecase1_driver_client_class: constructor' %
                      (self._action_name))

        self._client = actionlib.SimpleActionClient(
            self._action_name, PerformUseCaseAction)

    def run_client(self):

        rospy.loginfo('%s: Waiting until the action server has started up....' %
                      (self._action_name))

        # Waits until the action server has started up and started
        # listening for goals.
        self._client.wait_for_server()

        rospy.loginfo('%s: action server has started up and started up and listening for goals.....' %
                      (self._action_name))

        # Creates a goal to send to the action server.
        self._goal = PerformUseCaseGoal(
            usecase=self._action_name, operation='start')

        # Sends the goal to the action server.
        self._client.send_goal(self._goal,
                               active_cb=self.callback_active,
                               feedback_cb=self.callback_feedback,
                               done_cb=self.callback_done)

        rospy.loginfo("Goal has been sent to the action server.")
        # # Waits for the server to finish performing the action.
        # self._client.wait_for_result()

        # return self._client.get_result()

    def callback_active(self):
        rospy.loginfo("Action server is processing the goal")

    def callback_done(self, state, result):
        rospy.loginfo("Action server is done. State: %s, result: %s" %
                      (str(state), str(result)))

    def callback_feedback(self, feedback):
        rospy.loginfo("Feedback:%s" % str(feedback))
