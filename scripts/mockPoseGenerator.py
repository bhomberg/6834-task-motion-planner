from random import shuffle
import numpy

# TODO: Change to message type instead of dict, also add second action

class MockPoseGenerator:
    def __init__(self):
        self.state = dict()

    def next(self, action):
        if action in self.state:
            # if there are letters left, return it
            if len(self.state[action]) > 0:
                return self.state[action].pop()
            else:
                return None
        else:
            # generate list of poses
            if action == 'PICKUP':
                self.state[action] = ['N','NE','E','SE','S','SW','W','NW']
                shuffle(self.state[action])
            else:
                x_poses =  list(xrange(17))
                y_poses = list(xrange(17))
                shuffle(x_poses)
                shuffle(y_poses)
                self.state[action]=numpy.transpose([x_poses,y_poses]).tolist()
            # randomize the order
            # return a pose
            return self.state[action].pop()

    def reset(self, action):
        if action in self.state:
            del self.state[action]

if __name__ == "__main__":
    m = MockPoseGenerator()
    for i in range(9):
        print m.next('PICKUP')
    for i in range(18):
        print m.next('PUTDOWN')
    m.reset('PICKUP')
    m.reset('PUTDOWN')
    print m.next('PICKUP')
    print m.next('PUTDOWN')
