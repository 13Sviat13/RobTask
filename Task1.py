import numpy as np

class Robot:
    def __init__(self, world, P, S, path, initial_position=(0, 0)):
        self.world = world  # 2D world matrix
        self.P = P  # Probability of correct color detection
        self.S = S  # Probability of moving
        self.path = path  # Robot's movement path
        self.m, self.n = len(world), len(world[0])  # Dimensions of the world
        self.current_position = initial_position  # Robot's initial position

        # Initialize probabilities
        self.probabilities = np.full((self.m, self.n), 1 / (self.m * self.n))  # Uniform distribution

    def scan(self, measurement):
        # Update probabilities based on measurement
        new_probabilities = np.zeros_like(self.probabilities)
        for i in range(self.m):
            for j in range(self.n):
                if self.world[i][j] == measurement:
                    new_probabilities[i][j] = self.probabilities[i][j] * self.P
                else:
                    new_probabilities[i][j] = self.probabilities[i][j] * (1 - self.P)
        self.probabilities = new_probabilities
        self.normalize()

    def normalize(self):
        total = np.sum(self.probabilities)
        if total > 0:
            self.probabilities /= total

    def move(self, direction):
        # Simulate movement
        new_probabilities = np.zeros_like(self.probabilities)

        for i in range(self.m):
            for j in range(self.n):
                # Move with probability S
                if direction == 'L':
                    new_probabilities[i][(j - 1) % self.n] += self.probabilities[i][j] * self.S
                elif direction == 'R':
                    new_probabilities[i][(j + 1) % self.n] += self.probabilities[i][j] * self.S
                elif direction == 'U':
                    new_probabilities[(i - 1) % self.m][j] += self.probabilities[i][j] * self.S
                elif direction == 'D':
                    new_probabilities[(i + 1) % self.m][j] += self.probabilities[i][j] * self.S

                # Stay in place
                new_probabilities[i][j] += self.probabilities[i][j] * (1 - self.S)

        self.probabilities = new_probabilities
        self.normalize()

    def execute_path(self, measurements):
        for direction, measurement in zip(self.path, measurements):
            self.move(direction)
            self.scan(measurement)

# World 4x7
world = [
    ['R', 'G', 'R', 'G', 'G', 'R', 'G'],
    ['R', 'G', 'G', 'G', 'R', 'R', 'G'],
    ['G', 'R', 'G', 'R', 'R', 'G', 'R'],
    ['R', 'G', 'R', 'G', 'R', 'R', 'R']
]

# Create the robot
robot = Robot(world, 0.75, 0.6, ['L', 'L', 'D', 'R', 'U', 'U', 'R', 'D'], initial_position=(0, 0))

# Measurements corresponding to each step in the path
measurements = ['R', 'G', 'R', 'R', 'R', 'G', 'R', 'G']

# Execute the path
robot.execute_path(measurements)

# Final probability distribution
print("Final Probability Distribution:")
print(robot.probabilities)