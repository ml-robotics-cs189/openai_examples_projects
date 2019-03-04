import random

class QQLearn:

    def __init__(self, actions, epsilon, alpha, gamma):
        self.q1 = {}
        self.q2 = {}
        self.epsilon = epsilon  # exploration constant
        self.alpha = alpha      # discount constant
        self.gamma = gamma      # discount factor
        self.actions = actions

    def getQ1(self, state, action):
        return self.q1.get((state, action), 0.0)

    def getQ2(self, state, action):
    	return self.q2.get((state, action), 0.0)

    def learnQQ(self, state, action, reward, value1, value2):
        '''
        Q-learning:
            Q(s, a) += alpha * (reward(s,a) + max(Q(s') - Q(s,a))
        '''
        oldv_1 = self.q1.get((state, action), None)
        oldv_2 = self.q2.get((state, action), None)

        if oldv_1 or oldv_2 is None:
            self.q1[(state, action)] = reward
            self.q2[(state, action)] = reward

        else:
            self.q1[(state, action)] = oldv_1 + self.alpha * (value1 - oldv_1)
            self.q2[(state, action)] = oldv_2 + self.aplha * (value2 - oldv_2)

	
    def chooseAction(self, state, return_q=False):
        q1 = [self.getQ1(state, a) for a in self.actions]
        q2 = [self.getQ2(state, a) for a in self.actions]

        maxQ1 = max(q1)
        maxQ2 = max(q2)

        if random.random() < self.epsilon:

        	if random.random() < 0.5:
        		minQ1 = min(q1)
        		#not sure what mag is?
        		mag1 = max(abs(minQ1), abs(maxQ1))
        		#also don't know where i is coming from
        		q1 = [q1[i] + random.random() * mag1 - 0.5 * mag1 for i in range(len(self.actions))]
        		maxQ1 = max(q1)

        	else: 

        		minQ2 = min(q2)
        		mag2 = max(abs(minQ2), abs(maxQ2))

        		q2 = [q2[i] + random.random() * mag2 - 0.5 * mag2 for i in range(len(self.actions))]
        		maxQ2 = max(q2)


        if random.random() < 0.5:

        	count = q1.count(maxQ1)

        	if count > 1:
        		best = [i for i in range(len(self.actions)) if q1[i] == maxQ1]
        		i = random.choice(best)

        	else:
        		i = q1.index(maxQ1)

        else:

        	count = q2.count(maxQ2)

        	if count > 1:
        		best = [i for i in range(len(self.actions)) if q2[i] == maxQ2]
        		i = random.choice(best)

        	else:
        		i = q2.index(maxQ2)


        action = self.actions[i]        
        if return_q: # if they want it, give it!
            return action, q1, q2
        return action

    def learn(self, state1, action1, reward, state2):
        max_q1 = max([self.getQ1(state2, a) for a in self.actions])
		max_q2 = max([self.getQ2(state2, a) for a in self.actions])
        self.learnQQ(state1, action1, reward, reward + self.gamma*max_q1, self.gamma*max_q2)

