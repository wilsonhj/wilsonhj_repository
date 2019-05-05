# this code is adapted from Code for the book "Artificial Intelligence: A Modern Approach"
# https://github.com/aimacode

class Problem():
    def __init__(self, input_graph, heuristic = {}, initial='S', goal='G'):
        self.graph = input_graph
        self.initial_state = 'S'
        self.goal_state = 'G'
        self.heuristic = heuristic

    def actions(self, state):
        """ in a graph problems, action is simply 
        the neighbor nodes from the current node"""
        try:
            return sorted(list(self.graph[state].keys()))       # children are sorted in lexicographic order
        except:
            return None

    def result(self, state, action):
        """ this is redundant function in graph problems 
        because action is simply directing the next state"""
        return action

    def path_cost(self, cost_so_far, s1, action, s2):
        try:
            return cost_so_far + self.graph[s1][s2]
        except:
            return float('inf')     # in minimization, inf encodes impossible situation (constraints)

    def h(self, state):
        try:
            return self.heuristic[state]
        except:
            return None

    def goal_test(self, state):
        return state == 'G'

class SearchNode():
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def child_node(self, problem, action):
        next_state = problem.result(self.state, action)
        if next_state:
            return SearchNode(next_state, self, action, problem.path_cost(self.path_cost, self.state, action, next_state))
        else:
            return None

    def expand(self, problem):
        return [self.child_node(problem, action) for action in problem.actions(self.state) if action]

    def path(self):
        """ path from the root search node to the current node """
        node = self
        path = []
        while node:
            path.append(node)
            node = node.parent
        return list(reversed(path))

    def solution(self):
        """ sequence of actions from the root to the current node """
        return [node.action for node in self.path()[1:]]    # the initial state has no action

    def __repr__(self):
        return "<Node:state={}>".format(self.state)


# tree_search means that the search algorithm internally builds a tree of nodes
# so, it does not check whether you visited the same state before by alternative way from the input graph
# you can turn tree search to graph search by remembering what nodes you visited (explored set), and
# do a test to a new node generated before putting it into the queue.


def breadth_first_tree_search(problem, demo=False):
    print("\n\nSTART breadth_first_tree_search")
    iteration = 1
    frontier = [SearchNode(problem.initial_state, parent=None, action=None, path_cost=0)]
    while frontier:
        print("\niter:{}\n\tfrontier_before_pop:{}".format(iteration, frontier))
        node = frontier.pop(0)      # first-in-first-out, i.e., Queue
        print("\tpop node:{}\n\tfrontier_after_pop:{}".format(node, frontier))

        # do goal test after pop because BFS only finds a valid path to the goal
        goal_test_result =problem.goal_test(node.state)
        print("\tgoal_test on node:{} -> {}".format(node, goal_test_result))
        if goal_test_result:
            print("\n\nEND breadth_first_tree_search\n\n")
            return node

        # expand the current node and generate children nodes
        children_nodes = node.expand(problem)
        print("\texpand children:{}".format(children_nodes))

        # add children_nodes to the end of the current frontier nodes
        frontier = frontier + children_nodes
        iteration += 1
        if demo:
            input()
    print("\n\nEND breadth_first_tree_search\n\n")
    return None


def depth_first_tree_search(problem, demo=False):
    print("\n\nSTART depth_first_tree_search")
    iteration = 1
    frontier = [SearchNode(problem.initial_state, parent=None, action=None, path_cost=0)]
    while frontier:
        # this is the only different line compared to breath_first_tree_search
        print("\niter:{}\n\tfrontier_before_pop:{}".format(iteration, frontier))
        node = frontier.pop()  # last-in-first-out, i.e., Stack
        print("\tpop node:{}\n\tfrontier_after_pop:{}".format(node, frontier))

        # do goal test after pop because BFS only finds a valid path to the goal
        goal_test_result =problem.goal_test(node.state)
        print("\tgoal_test on node:{} -> {}".format(node, goal_test_result))
        if goal_test_result:
            print("\n\nEND depth_first_tree_search\n\n")
            return node

        # expand the current node and generate children nodes
        children_nodes = node.expand(problem)
        print("\texpand children:{}".format(children_nodes))

        # add children_nodes to the end of the current frontier nodes
        children_nodes.reverse()        # to visit nodes in lexicographic order
        frontier.extend(children_nodes)
        iteration += 1
        if demo:
            input()
    print("\n\nEND depth_first_tree_search\n\n")
    return None


def uniform_cost_tree_search(problem, demo=False):
    print("\n\nSTART uniform_cost_tree_search")
    iteration = 1
    frontier = [SearchNode(problem.initial_state, parent=None, action=None, path_cost=0)]

    while frontier:
        print("\niter:{}\n\tfrontier_before_pop:{}".format(iteration, frontier))
        node = frontier.pop(0)  # last-in-first-out, i.e., Stack
        print("\tpop node:{}\n\tfrontier_after_pop:{}".format(node, frontier))

        # do goal test after pop because BFS only finds a valid path to the goal
        goal_test_result =problem.goal_test(node.state)
        print("\tgoal_test on node:{} -> {}".format(node, goal_test_result))
        if goal_test_result:
            print("\n\nEND uniform_cost_tree_search\n\n")
            return node

        # expand the current node and generate children nodes
        children_nodes = node.expand(problem)
        frontier.extend(children_nodes)
        # sort the queue by the total path cost; you can actually use heap instead of list for this
        frontier.sort(key=lambda n: n.path_cost)

        iteration += 1
        if demo:
            input()
    print("\n\nEND depth_first_tree_search\n\n")
    return None


def a_star_tree_search(problem, demo=False):
    print("\n\nSTART a_star_tree_search")
    iteration = 1
    frontier = [SearchNode(problem.initial_state, parent=None, action=None, path_cost=0)]

    while frontier:
        print("\niter:{}\n\tfrontier_before_pop:{}".format(iteration, frontier))
        node = frontier.pop(0)  # last-in-first-out, i.e., Stack
        print("\tpop node:{}\n\tfrontier_after_pop:{}".format(node, frontier))

        # do goal test after pop because BFS only finds a valid path to the goal
        goal_test_result =problem.goal_test(node.state)
        print("\tgoal_test on node:{} -> {}".format(node, goal_test_result))
        if goal_test_result:
            print("\n\nEND a_star_tree_search\n\n")
            return node

        # expand the current node and generate children nodes
        children_nodes = node.expand(problem)
        frontier.extend(children_nodes)
        # sort the queue by the total path cost; you can actually use heap instead of list for this
        frontier.sort(key=lambda n: n.path_cost + problem.h(n.state))

        iteration += 1
        if demo:
            input()
    print("\n\nEND a_star_tree_search\n\n")
    return None




if __name__ == "__main__":
    # Excercies problem given in the lecture
    # We define an input problem by a graph using dictionary
    # for example,
    # all neighbors from S can be retrived by Graph_Problem['S'].keys()
    # an edge from S to A with path cost 3 can be retrieved by Graph_Problem['S']['A']
    # you could replace the following graph with the one given in other problems.
    graph_problem = {
        'S': {'A': 3, 'B': 2, 'C': 1},
        'A': {'D': 6}, # 'A':{'A': 15, 'B': 2, 'G': 150}
        'B': {'E': 4},
        'C': {'G': 20},
        'D': {'F': 1},
        'E': {'G': 8},
        'F': {'G': 1}
    }
    heuristic_table = {
        'S':9, # estimate remaining cost from current to goal
        'A':5, # estimate
        'B':4,
        'C':8,
        'D':1,
        'E':3,
        'F':0,
        'G':0
    }

    problem = Problem(graph_problem, heuristic_table, initial='S', goal='G')
    # bfs_solution_node = breadth_first_tree_search(problem, True)
    # print("BFS solution:{}".format(['S'] + bfs_solution_node.solution()))

    # dfs_solution_node = depth_first_tree_search(problem, True)
    # print("DFS solution:{}".format(['S'] + dfs_solution_node.solution()))

    # ucs_solution_node = uniform_cost_tree_search(problem, True)
    # print("UCS solution:{}".format(['S'] + ucs_solution_node.solution()))

    astar_solution_node = a_star_tree_search(problem, True)
    print("A* solution:{}".format(['S'] + astar_solution_node.solution()))
