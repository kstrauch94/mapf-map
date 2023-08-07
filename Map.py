from collections import defaultdict, deque
from clingo import Function, Number
class Node:

    def __init__(self, name) -> None:
        self.name = name
        self.k_agent = defaultdict(lambda: None)
        self.k = None

    def set_k_agent(self, k, agent):
        self.k_agent[agent] = k

    def set_k(self, k):
        self.k = k

    def __str__(self) -> str:
        return str(self.name)

    def __repr__(self) -> str:
        return self.__str__()

class Map:

    def __init__(self, edges=None) -> None:
        self.edges = defaultdict(set)
        self.symbol_to_node = {}
        self.add_edges(edges)
        self.agents = {}
        self.agent_dist = {}
        
        # agent shortest paths in time order
        # only populated when calling the shortest path function of this class
        self.agent_sps = {}

        self.corridors_agent = defaultdict(lambda: defaultdict(set))

        self.corridors = defaultdict(set)


    def add_edges(self, edges):
        if edges is not None:
            for edge in edges:
                self.add_edge(*edge)

    def add_edge(self, a, b):
        self.edges[a].add(b)
        self.edges[b].add(a)


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
        return self.agent_sp[agent]

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

    def set_initial_corridor(self, ctl=None):
        # if no control is given or there are no spath atoms that set the corridor
        # get SP of each robot and set it as corridor 0
        if ctl is None or len(list(ctl.symbolic_atoms.by_signature("spath", 4))) == 0:
            for agent in self.agents.keys():
                path = self.get_shortest_path(agent)
                for node in path:
                    self.corridors_agent[agent][0].add(node)
                    self.corridors[0].add(node)
                    node.set_k_agent(0, agent)
                    node.set_k(0)
        else:
            for atom in ctl.symbolic_atoms.by_signature("spath", 4):
                agent = atom.symbol.arguments[0].number
                vertex1 = self.symbol_to_node[atom.symbol.arguments[1]]
                vertex2 = self.symbol_to_node[atom.symbol.arguments[2]]

                # add spath to corridor 0
                self.corridors_agent[agent][0].add(vertex1)
                self.corridors[0].add(vertex1)
                vertex1.set_k_agent(0, agent)
                vertex1.set_k(0)

                self.corridors_agent[agent][0].add(vertex2)
                self.corridors[0].add(vertex2)
                vertex2.set_k_agent(0, agent)
                vertex2.set_k(0)

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

    def k_corridors_agents(self, k_corr):
        for k in range(1, k_corr+1):
            for agent, corridor in self.corridors_agent.items():
                if k in corridor:
                    continue
                for node in corridor[k-1]:
                    for neighbor in self.edges[node]:
                        if neighbor.k_agent[agent] is None:
                            corridor[k].add(neighbor)
                            neighbor.set_k_agent(k, agent)
                
    def k_corridors(self, k_corr):
        for k in range(1, k_corr+1):
            if k in self.corridors:
                continue

            for node in self.corridors[k-1]:
                for neighbor in self.edges[node]:
                    if neighbor.k is None:
                        self.corridors[k].add(neighbor)
                        neighbor.set_k(k)
            
    def atoms_in_corridor_upto_k(self, maxk):
        atoms = []
        nodes = self.nodes_in_corridor_upto_k(maxk)
        #print(f"nodes: {len(nodes)} in k {maxk}")
        for node in nodes:
            for neighbor in self.edges[node]:
                if neighbor in nodes:
                    atoms.append(f"edge({node},{neighbor}).")

            atoms.append(f"vertex({str(node)}).")

        return atoms

    def nodes_in_corridor_upto_k(self, maxk):
        nodes = []
        for k in range(0,maxk+1):
            for node in self.corridors[k]:
                nodes.append(node)

        return nodes
    
    def nodes_in_corridor_upto_k_agent(self, maxk, agent):
        nodes = []
        for k in range(0,maxk+1):
            for node in self.corridors_agent[agent][k]:
                nodes.append(node)

        return nodes

    def create_instance(self, k):
        atoms = self.atoms_in_corridor_upto_k(k)
        for agent, s_g in self.agents.items():
            atoms.append(f"start({agent},{s_g['s']}).")
            atoms.append(f"goal({agent},{s_g['g']}).")
            atoms.append(f"agent({agent}).")

            atoms.append(f"agent_SP({agent},{self.shortest_path_dist(agent)}).")
        return atoms

    def is_corridor_reachable(self, k, horizon):
        # make sure that corridors are generated up to k (if possible)
        # before the check
        self.k_corridors(k)
        self.k_corridors_agents(k)

        # if k is still not in the list then it is above the max
        if k not in self.corridors:
            return False
        
        # if any node in the corridor is reachable for any agent then the corridor is reachable
        for agent in self.agents.keys():
            for node in self.corridors[k]:
                if self.reachable_node(node, agent, horizon):
                    return True
        
        return False

    def reachable_nodes_at_times(self, max_k, agent, delta, agent_specific=False):
        horizon = self.shortest_path_dist(agent)+delta

        if agent_specific:
            nodes = self.nodes_in_corridor_upto_k_agent(max_k, agent)
        else:
            nodes = self.nodes_in_corridor_upto_k(max_k)

        reachable = {}

        reachable[agent, 0] = [self.agents[agent]["s"]]

        for time in range(1, horizon+1):
            reachable[agent, time] = set()

            for node in reachable[agent, time-1]:
                # look at neighbors
                for neighbor in self.edges[node]:
                    if neighbor not in nodes:
                        continue
                    skip = False
                    for ag in self.agents.keys():
                        if ag == agent:
                            continue
                        if self.agents[ag]["g"] == neighbor and self.shortest_path_dist(ag)+delta <= time:
                            skip = True
                            break

                    if not skip and self.node_goal_distance(neighbor, agent) <= horizon - time:
                        reachable[agent, time].add(neighbor)
                # if can wait
                skip = False
                for ag in self.agents.keys():
                    if ag == agent:
                        continue
                    if self.agents[ag]["g"] == node and self.shortest_path_dist(ag)+delta <= time:
                        skip = True
                        break
                if not skip and self.node_goal_distance(node, agent) <= horizon - time:
                        reachable[agent, time].add(node)
                        
        atoms = []
        for key, value in reachable.items():
            agent, time = key
            for node in value:
                atoms.append(f"poss_loc({agent},{node},{time}).")
        
        return atoms

    def reachable_nodes_at_times_makespan(self, max_k, agent, horizon, agent_specific=False):

        if agent_specific:
            nodes = self.nodes_in_corridor_upto_k_agent(max_k, agent)
        else:
            nodes = self.nodes_in_corridor_upto_k(max_k)

        reachable = {}

        reachable[agent, 0] = [self.agents[agent]["s"]]

        for time in range(1, horizon+1):
            reachable[agent, time] = set()

            for node in reachable[agent, time-1]:
                # look at neighbors
                for neighbor in self.edges[node]:
                    if neighbor not in nodes:
                        continue

                    if self.node_goal_distance(neighbor, agent) <= horizon - time:
                        reachable[agent, time].add(neighbor)

                # if can wait
                if self.node_goal_distance(node, agent) <= horizon - time:
                        reachable[agent, time].add(node)
                        
        atoms = []
        for key, value in reachable.items():
            agent, time = key
            for node in value:
                atoms.append(f"poss_loc({agent},{node},{time}).")
        
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
        
    def corridor_atoms(self, k):
        atoms = ""
        for pos in self.corridors[k]:
            atoms += f"kpos({k},{pos}).\n"

        return atoms

    def corridor_atoms_alt(self, k):
        atoms = ""
        for node in self.corridors[k]:
            for agent in self.agents.keys():
                atoms += f"corridor({agent},{node},{k}).\n"

        return atoms

    def corridor_atoms_agents(self, k):
        atoms = ""
        for agent, corridors in self.corridors_agent.items():
            for pos in corridors[k]:
                atoms += f"kpos({k},{pos},{agent}).\n"

        return atoms
    
    def corridor_atoms_agents_alt(self, k):
        atoms = ""
        for agent, corridors in self.corridors_agent.items():
            for node in corridors[k]:
                atoms += f"corridor({agent},{node},{k}).\n"

        return atoms


    def teg(self, horizon, k):
        atoms = set()
        for node in self.corridors[k]:
            #print(node.name.arguments)
            for agent in self.agents.keys():
                # nodes can't reach the goal if they get there too late
                if self.node_distance(node, agent) > horizon:
                    for time in range(0, horizon+1):
                        atoms.add((Function("at",[Number(agent),Function("",node.name.arguments),Number(time)]), False))
                        continue
                    
                diff = horizon - self.node_distance(node, agent)
                start_dist = self.node_start_distance(node, agent)
                for time in range(start_dist+diff+1, horizon+1):
                        atoms.add((Function("at",[Number(agent),Function("",node.name.arguments),Number(time)]), False))

        return atoms
    
    def all_teg(self, horizon, max_k):
        atoms = set()
        for k in range(0,max_k+1):
            atoms.update(self.teg(horizon, k))
        print(len(atoms))
        return atoms

    def shortest_path_sum(self):
        sum = 0
        for agent in self.agents.keys():
            sum += self.shortest_path_dist(agent)

        return sum

    def print_sp_per_agent(self):
        for agent in self.agents.keys():
            print("SP", agent, self.shortest_path_dist(agent))
        
    def min_horizon(self):
        return max([self.shortest_path_dist(agent) for agent in self.agents.keys()])
