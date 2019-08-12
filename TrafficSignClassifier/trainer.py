import tensorflow as tf
from enum import Enum


class OptimizerName(Enum):
    ADAM = 1
    SGD = 2

class Trainer:
    def __init__(self, optimizer_name, learning_late):
        self.optimizer = self.__get_optimizer__(optimizer_name, learning_late)

    def __get_optimizer__(self, optimizer_name, learning_late):
        if optimizer_name == OptimizerName.ADAM:
            return tf.train.AdamOptimizer(learning_late)
        elif optimizer_name == OptimizerName.SGD:
            return tf.train.GradientDescentOptimizer(learning_late)
