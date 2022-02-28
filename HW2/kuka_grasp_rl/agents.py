import tensorflow as tf
import tensorflow_probability as tfp
import numpy as np


class ActorCriticAgent():

    def __init__(self, model):
        super(ActorCriticAgent, self).__init__()
        self.model = model
        self.huber_loss = tf.keras.losses.Huber(reduction=tf.keras.losses.Reduction.SUM)

    def act(self, state):
        """Select an action based on the policy output.
        Return: the action, the action log probabilities of this action sample
        and the value from the policy output"""

        # Hint: we are working with a discrete version of the Kuka grasping env
        # so you can use a Categorical distribution.

        ########## Code starts here ##########

        ########## Code ends here ##########
        return int(action[0]), action_log_probs, value

    def compute_loss(self, log_probs, returns, values):
        """Compute the actor loss and the critic loss separately and add them together."""

        ########## Code starts here ##########

        ########## Code ends here ##########
        return total_loss

    def compute_expected_return(self, rewards, gamma):
        """Compute the expected returns."""

        # Hint #1: Start from the rewards in reverse, it'll make your calculation easier to code.
        # Hint #2: Don't forget to normalize your returns in the end by subtracting the mean and dividing by the std plus some epsilon to avoid any divisions by 0.
        
        ########## Code starts here ##########

        ########## Code ends here ##########

        return returns
