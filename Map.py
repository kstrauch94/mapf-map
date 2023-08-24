from collections import defaultdict, deque
from clingo import Function, Number

class Node:

    def __init__(self, name) -> None:
        self.name = name

    def __str__(self) -> str:
        return str(self.name)

    def __repr__(self) -> str:
        return self.__str__()


class Map:

    def __init__(self, edges=None) -> None:
        self.edges = defaultdict(set)
        self.nodes = set()

        self.symbol_to_node = {}
        #self.node_to_symbol = {}

        self.add_edges(edges)

        self.agents = {}
        self.agent_dist = {}

    @property
    def agents(self):
        return self.agents.keys()

    def add_edges(self, edges):
        if edges is not None:
            for edge in edges:
                self.add_edge(*edge)

    def add_edge(self, a, b):
        self.nodes.add(a)
        self.nodes.add(b)

        self.edges[a].add(b)

    def get_edges(self, node):
        return self.edges[node]
    
    def bfs(self, start):
        parent = {start: None}
        dist = {start: 0}

        queue = deque()
        queue.append(start)

        while queue:
            _next = queue.popleft()

            for node in self.edges[_next]:
                if node not in dist:
                    parent[node] = _next
                    dist[node] = dist[_next] + 1

                    queue.append(node)

        return parent, dist

    def get_shortest_path(self, agent):
        parents, dist = self.bfs(self.agents[agent]["s"])
        path = []
        node = self.agents[agent]["g"]
        while node is not None:
            path.append(node)
            node = parents[node]

        self.agent_sps[agent] = path[::-1]
        return self.agent_sps[agent]

    def from_control(self, ctl):

        for atom in ctl.symbolic_atoms.by_signature("edge", 2):
            # grab the atoms for the corridors and save them
            a = atom.symbol.arguments[0]
            if a not in self.symbol_to_node:
                anode = Node(a)
                self.symbol_to_node[a] = anode
            else:
                anode = self.symbol_to_node[a]

            b = atom.symbol.arguments[1]
            if b not in self.symbol_to_node:
                bnode = Node(b)
                self.symbol_to_node[b] = bnode
            else:
                bnode = self.symbol_to_node[b]

            self.add_edge(anode,bnode)

        for atom in ctl.symbolic_atoms.by_signature("goal", 2):
            agent = atom.symbol.arguments[0].number
            goal = self.symbol_to_node[atom.symbol.arguments[1]]

            self.agents[agent] = {"g": goal}

        for atom in ctl.symbolic_atoms.by_signature("start", 2):
            agent = atom.symbol.arguments[0].number
            start = self.symbol_to_node[atom.symbol.arguments[1]]

            self.agents[agent]["s"] = start

        for agent, start_goal in self.agents.items():
            parents, dist = self.bfs(start_goal["s"])
            self.agent_dist[agent] = {"s": dist}
            parents, dist = self.bfs(start_goal["g"])
            self.agent_dist[agent]["g"] = dist

    def node_start_distance(self, node, agent):
        return self.agent_dist[agent]["s"][node]
    
    def node_goal_distance(self, node, agent):
        return self.agent_dist[agent]["g"][node]

    def shortest_path_dist(self, agent):
        return self.node_start_distance(self.agents[agent]["g"], agent)

    def node_distance(self, node, agent):
        return self.node_start_distance(node, agent) + self.node_goal_distance(node, agent)

    def reachable_node(self, node, agent, horizon):
        return self.node_distance(node, agent) <= horizon
    
    def check_other_agents_at_goal(self, agent, time, node, delta):
        ## check if the node is the goal of an agent
        # if so, check that the time point is before the agent's horizon
        for ag in self.agents.keys():
            if ag == agent:
                continue
            # if the current time is already over the horizon of the agent
            # then skip this node
            if self.agents[ag]["g"] == node and self.shortest_path_dist(ag)+delta <= time:
                return True
            
        return False

    # TODO: add possible list of nodes where reachability is limited (e.g. corridor of agents)
    def reachable_nodes_at_times(self, agent, delta, reachability_name="poss_loc"):
        horizon = self.shortest_path_dist(agent)+delta

        reachable = {}

        nodes = set()

        reachable[agent, 0] = [self.agents[agent]["s"]]

        for time in range(1, horizon+1):
            reachable[agent, time] = set()

            for node in reachable[agent, time-1]:
                # look at neighbors
                for neighbor in self.edges[node]:

                    skip = self.check_other_agents_at_goal(agent, time, neighbor, delta)

                    # if the agent can reach the goal in time from that node
                    # add it to the reachable nodes
                    if not skip and self.node_goal_distance(neighbor, agent) <= horizon - time:
                        reachable[agent, time].add(neighbor)
                        nodes.add(neighbor)

                # check if agent can stay in the node
                skip = self.check_other_agents_at_goal(agent, time, node, delta)
                if not skip and self.node_goal_distance(node, agent) <= horizon - time:
                        reachable[agent, time].add(node)
                        nodes.add(node)
                        
        atoms = []
        for key, value in reachable.items():
            agent, time = key
            for node in value:
                atoms.append(f"{reachability_name}({agent},{node},{time}).")
        
        return atoms

    def reachable_nodes_at_times_makespan(self, agent, horizon, reachability_name="poss_loc"):

        reachable = {}

        nodes = set()

        reachable[agent, 0] = [self.agents[agent]["s"]]

        for time in range(1, horizon+1):
            reachable[agent, time] = set()

            for node in reachable[agent, time-1]:
                # look at neighbors
                for neighbor in self.edges[node]:

                    if self.node_goal_distance(neighbor, agent) <= horizon - time:
                        reachable[agent, time].add(neighbor)
                        nodes.add(neighbor)

                # if can wait
                if self.node_goal_distance(node, agent) <= horizon - time:
                        reachable[agent, time].add(node)
                        nodes.add(node)
                        
        atoms = []
        for key, value in reachable.items():
            agent, time = key
            for node in value:
                atoms.append(f"{reachability_name}({agent},{node},{time}).")
        
        return atoms, nodes
    
    def nodes_to_graph_atoms(self, nodes, edge_name="edge", vertex_name="vertex"):
        atoms = set()
        for node in nodes:
            # add edges to nodes in the corridor
            for neighbor in self.get_edges(node):
                if neighbor in nodes:
                    atoms.add(f"{edge_name}({node},{neighbor}).")

            # add ndoes
            atoms.add(f"{vertex_name}({str(node)}).")

        return atoms
    
    def create_instance(self, agents, start_name="start", goal_name="goal", agent_name="agent", agent_sp_name="agent_SP"):
        atoms = set()
        for agent in agents:
            s_g = self.agents[agent]
            atoms.add(f"{start_name}({agent},{s_g['s']}).")
            atoms.add(f"{goal_name}({agent},{s_g['g']}).")
            atoms.add(f"{agent_name}({agent}).")

            atoms.add(f"{agent_sp_name}({agent},{self.shortest_path_dist(agent)}).")
        return atoms

    def shortest_path_sum(self):
            return sum([self.shortest_path_dist(agent) for agent in self.agents])

    def min_horizon(self):
        return max([self.shortest_path_dist(agent) for agent in self.agents])
    

class Corridor:

    def __init__(self, map, path=None):
        self.map = map

        self.corridor = defaultdict(set)

        if path is not None:
            self.path = path
        else:
            self.path = self.map.get_shortest_path()

        for node in self.path:
            self.corridor[0].add(node)
        
    def reset(self, path):
        # there should not be a reason to reset without a new path
        self.corridor = defaultdict(set)
        
        self.path = path
        for node in self.path:
            self.corridor[0].add(node)

    def nodes_in_corridor(self, k):
        nodes = set()
        for i in range(0, max(k, self.corridor.keys()) +1):
            if i in self.corridor:
                nodes = nodes.union(self.corridor[i])

        return nodes

    def k_corridors(self, k_corr):
        for k in range(1, k_corr+1):
            if k in self.corridor:
                continue

            cummulative_corridor = self.nodes_in_corridor(k-1)

            for node in self.corridor[k-1]:
                for neighbor in self.map.get_edges(node):
                    if neighbor not in cummulative_corridor:
                        self.corridor[k].add(neighbor)
                            

    def atoms_in_corridor(self, maxk, edge_name="edge", vertex_name="vertex"):
        atoms = set()
        self.k_corridors(maxk)
        nodes = self.nodes_in_corridor(maxk)

        for node in nodes:
            # add edges to nodes in the corridor
            for neighbor in self.map.get_edges(node):
                if neighbor in nodes:
                    atoms.add(f"{edge_name}({node},{neighbor}).")

            # add ndoes
            atoms.add(f"{vertex_name}({str(node)}).")

        return atoms
    
    
    def max_corridor(self, horizon):
        reachable = True
        k = 0
        while reachable:
            k += 1
            self.k_corridors(k)
            reachable = self.is_corridor_reachable(k, horizon)

        self.k_corridors_agents(k-1)
        return k - 1
    
    def path_to_atoms(self, agent, at_name="at", move_name="move"):
        atoms = []
        prev = None
        for i, node in enumerate(self.path):
            atoms.append(f"{at_name}({agent},{str(node)},{i}).")
            if i != 0:
                atoms.append(f"{move_name}({agent},{str(prev)},{str(node)},{i}). ")
            
            prev = node

        return atoms
    

class CorridorManager:

    def __init__(self, map, paths=None):
        self.map = map

        self.corridors = {}

        for agent in self.map.agents:
            if paths is not None and agent in paths:
                agent_path = paths[agent]
            else:
                agent_path = self.map.get_shortest_path(agent)

            self.corridors[agent] = Corridor(self.map, agent_path)