import abc


class MonteCarlo(abc.ABC):
    """
    Implements a base class of Monte Carlo reinforment learning
    """
    @abc.abstractmethod
    def policy(self, state, other_params):
        pass

    @abc.abstractmethod
    def update_reward(self, state_action, reward):
        pass
