from random import shuffle

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
            # generate a random order & return a letter
            self.state[action] = ['A','B','C','D','E','F','G','H','I','J']
            shuffle(self.state[action])
            return self.state[action].pop()

    def reset(self, action):
        if action in self.state:
            del self.state[action]

if __name__ == "__main__":
    m = MockPoseGenerator()
    for i in range(11):
        print m.next('pick')
    m.reset('pick')
    print m.next('pick')