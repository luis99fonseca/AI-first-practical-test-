# Luís Fonseca
# nº 89066

from tree_search import *
import math


class NodeContainer(SearchNode):
    def __init__(self, state, parent, evalfun, cost = 0,depth = 0):
        super().__init__(state, parent)
        self.evalfunc = evalfun
        self.children = None
        self.cost = cost
        self.depth = depth

    def in_parent(self, state):
        if self.parent == None:
            return False
        return self.parent.state == state or self.parent.in_parent(state)

    def __str__(self):
        return f"no[ ({self.state}), parent:{self.parent}, depth: {self.depth} ]"
    def __repr__(self):
        return str(self)


class MyTree(SearchTree):

    def __init__(self,problem, strategy='breadth',max_nodes=None):
        self.problem = problem
        self.strategy = strategy
        self.root = NodeContainer( problem.initial, None, self.problem.domain.heuristic(self.problem.initial, self.problem.goal ), 0, 0)
        self.open_nodes = [self.root]
        self.max_nodes = max_nodes
        self.solution_cost = 0
        self.solution_length = 0
        self.total_nodes = 1

        self.terminal_nodes = 0
        self.non_terminal_nodes = 0

    def astar_add_to_open(self,lnewnodes):
        self.open_nodes = sorted(self.open_nodes + lnewnodes, key=lambda no:  no.evalfunc)

    #Based on http://ozark.hendrix.edu/~ferrer/courses/335/f11/lectures/effective-branching.html?fbclid=IwAR2SoOngh3XFMrUXdsKwXLc502MZ6tBLax8CXS_Ywsl_wdNNX08MfnPDQGQ
    #Nota: inner comments, are from the source above
    def effective_branching_factor(self):
        # Requires N and d
        N = self.total_nodes
        d = self.solution_length

        # Select an error tolerance
        error = 0.001
        b_factor = math.pow(N, 1 / d)   # Average the estimates to provide a guess for b*
        while True:

            # temp_N = 0
            # for i in range(d+1):    #Note: has to be d+1 as range its exclusive
            #     temp_N += math.pow(b_factor, i)

            # Calculate N' using the guess for b* and d
            temp_N = ( (math.pow(b_factor, d+1) - 1 ) / (b_factor - 1) ) #Nota: this one is faster; Formula encontra no slide 126

            # If abs(N' - N) > error, modify the low or high estimate accordingly; Otherwise, it is within the error, so return the guess for b*
            diff = temp_N - N
            if abs(diff) <= error:
                return b_factor
            b_factor += 0.000001 if diff < 0 else -0.000001
    def update_ancestors(self,node):
        if node is None:
            return
        if node.children:
            node.evalfunc = min(node.children,key =  lambda x : x.evalfunc).evalfunc
            self.update_ancestors(node.parent)

    def discard_worse(self):
        max_cost_parents = sorted(self.open_nodes, key = lambda x : x.parent.evalfunc, reverse=True)
        max_node = None
        for node in max_cost_parents:
            if self.has_leafs(node.parent):
                max_node = node.parent
                break

        # remove the children from open_nodes, as it is done, we decrement terminal_nodes count;
        for n in max_node.children:
            self.open_nodes.remove(n)
            self.terminal_nodes -= 1

        # appenting the parent node; thus it goes from beeing an non_terminal to beeing one
        self.open_nodes.append(max_node)
        self.terminal_nodes += 1
        self.non_terminal_nodes -= 1

    # checks if all n_node childs, are terminal, e.g leafs
    def has_leafs(self, n_node):
        for child_n in n_node.children:
            if child_n not in self.open_nodes:
                return False
        return True

    def search2(self):
        while self.open_nodes:  #Nota: pythonic

            node = self.open_nodes.pop(0)

            if self.problem.goal_test(node.state):
                self.solution_cost = node.cost
                self.solution_length = node.depth
                self.terminal_nodes += 1            #Nota: o ultimo é terminal, e como retornamos aqui, temos também que o contabilizar aqui
                return self.get_path(node)
            lnewnodes = []
            for a in self.problem.domain.actions(node.state):

                newstate = self.problem.domain.result(node.state, a)

                if not node.in_parent(newstate):
                    temp_cost = node.cost + self.problem.domain.cost(node.state, a)
                    newnode = NodeContainer(newstate, node, self.problem.domain.heuristic(newstate, self.problem.goal ) + temp_cost, temp_cost , node.depth + 1)
                    lnewnodes.append(newnode)
                    self.total_nodes += 1   #Nota: só se incrementa aqui porque só aqui é que se efetivamente incrementa o nº de nodes

            self.terminal_nodes += len(lnewnodes)

            node.children = lnewnodes
            self.update_ancestors(node)

            self.add_to_open(lnewnodes)

            while self.max_nodes and len(self.open_nodes) + self.non_terminal_nodes >= self.max_nodes:  #Nota: decidiu-se verificar aqui isto, para nao estar sempre a chamar a função de modo a reduzir "overhead"
                self.discard_worse()

            if len(
                    node.children) > 0:  # Nota: isto é, se tiver filhos, como lhe demos pop(ou open), então é nao terminal
                self.non_terminal_nodes += 1
                self.terminal_nodes -= 1
        return None


    # shows the search tree in the form of a listing
    def show(self,heuristic=False,node=None,indent=''):
        if node==None:
            self.show(heuristic,self.root)
            print('\n')
        else:
            line = indent+node.state
            if heuristic:
                line += (' [' + str(node.evalfunc) + ']')
            print(line)
            if node.children==None:
                return
            for n in node.children:
                self.show(heuristic,n,indent+'  ')


