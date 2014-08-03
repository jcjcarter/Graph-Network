""" Homework 2"""
import itertools
import comp182
import provided
import collections
import random

def brute_force_distance(g, startnode):
    """
    Computes the distance from a given start node to all
    the nodes in the graph.

    Arguments:
    g -- a dictionary representation of a  undirected graph.
    startnode -- a node with the graph g.

    Returns:
    A dictionary where the keys are nodes in g and the values are the distance
    from startnode to that node."""

    
    nodedistancedic = {} #empty dictioanry where the nodes' distance from start node will be placed

    for node in g.keys(): #for loop sets the default value of the distance to each node to unspecified and excludes starting node from subset
        if node != startnode:
            nodedistancedic[node] = None

    nodedistancedic[startnode] = 0 #the distance from the start node to itself is zero

    maxdiameter = g.keys()  #the largest possible distance between node i and any other node
    maxdiameter.remove(startnode)
    startnodepath = startnode #node i is the starting node of every path we consider

    counter = 1 #keeps track of the progrss of for loop when going over max max diameter

    for findnode in maxdiameter:
        subsetnodelist = g.keys() #the list where the nodes will be placed for permutations
        subsetnodelist.remove(startnode)
        subsetnodelist.remove(findnode)
        for length in xrange(len(maxdiameter)):
            for subset in itertools.combinations(subsetnodelist,length):
                for permutation in itertools.permutations(subset):
                    path = list(permutation)
                    path.insert(0,startnode)
                    path.append(findnode)
                    connected = True
                    for node in xrange(length+1):
                        if path[node+1] not in g[path[node]]:
                            connected = False
                            break;
                
                    if connected == True and nodedistancedic[findnode] == None:
                        #print "\n the node:", node, "\n where it can be found.", nodedistancedic[node]
                        nodedistancedic[findnode] = len(path) - 1
        counter += 1

    return nodedistancedic

""" Test graphs """
g1 = {0:set([1,2,3]),
      1:set([0,4]),
      2:set([0,5]),
      3:set([0,6]),
      4:set([1]),
      5:set([2]),
      6:set([3])}

g2 = {0:set([1,2,3,4,5,6,7,8]),
      1:set([0,2,8]),
      2:set([0,1,3]),
      3:set([0,2,4]),
      4:set([0,3,5]),
      5:set([0,4,6]),
      6:set([0,5,7]),
      7:set([0,8,6]),
      8:set([0,1,7])}

g3 = {0:set([1,2,3]),
      1:set([0,4,5]),
      2:set([0,6,3]),
      3:set([0,2,4,7]),
      4:set([8,3,5,1]),
      5:set([1,4,9]),
      6:set([2,10,7]),
      7:set([3,8,6,10]),
      8:set([4,11,7,9]),
      9:set([5,8,11]),
      10:set([6,7,11]),
      11:set([10,8,9])}

def random_upa_graphs():
    """ Implementation on random graphs of varying sizes that are
    generated under the undirected preferential attachment model.

    Argument:
    None

    Returns:
    a tuple containing the result of the run and number of nodes used."""
    first = provided.upa(2,2)
    second = provided.upa(4,2)
    third = provided.upa(6,2)
    fourth = provided.upa(8,2)
    fifth = provided.upa(10,2)
    sixth = provided.upa(12,2)
    
    print comp182.time_func(brute_force_distance, [first, 0]), "UPA(2,2)"
    print "Edges in graph:", edge_count(first)
    print comp182.time_func(brute_force_distance, [second, 0]),"UPA(4,2)"
    print "Edges in graph:", edge_count(second)
    print comp182.time_func(brute_force_distance, [third, 0]),"UPA(6,2)"
    print "Edges in graph:", edge_count(third)
    print comp182.time_func(brute_force_distance, [fourth, 0]),"UPA(8,2)"
    print "Edges in graph:", edge_count(fourth)
    print comp182.time_func(brute_force_distance, [fifth, 0]),"UPA(10,2)"
    print "Edges in graph:", edge_count(fifth)
    print comp182.time_func(brute_force_distance, [sixth, 0]),"UPA(12,2)"
    print "Edges in graph:", edge_count(sixth)

def bfs(g, startnode):
    """Finds the shortest path from a starting node to all the nodes in the
    graph.

    Arguments:
    g -- an undirected graph
    startnode --  a node which will be the starting point.

    Reutrns:
    A dictionary mappning nodes to distances, a dictionary mapping nodes to parent nodes, and
    a set of visited nodes."""

    emptyqueue = collections.deque() #initialize Q to an empty queue
    nodedistancedic = {} #this dictionary will keep map the distances from the starting node.
    nodeparentdic = {} #keeps track of the parent node
    for node in g.keys():
        nodedistancedic[node] = None
        nodeparentdic[node] = None
    nodedistancedic[startnode] = 0 #distance from start node to its self is zero
    nodesprocessed = set() #contains all nodes that have been completely processed
    emptyqueue.append(startnode)
    while len(emptyqueue) != 0:
        nodej = emptyqueue.popleft()
        for neighborh in g[nodej] - nodesprocessed:
            if nodedistancedic[neighborh] == None:
                nodedistancedic[neighborh] = nodedistancedic[nodej] + 1
                nodeparentdic[neighborh] = nodej
                emptyqueue.append(neighborh)
        nodesprocessed.add(nodej)
    return "Node Distance Dictionary:", nodedistancedic, "Node Parent Dictioanry:", nodeparentdic,"Nodes Processed:", nodesprocessed
                
def bfs_random_upa_graphs():
    """ Implementation on random graphs of varying sizes that are
    generated under the undirected preferential attachment model
    using the BFS algorithm.

    Argument:
    None

    Returns:
    a tuple containing the result of the run and number of nodes used."""
    first = provided.upa(500,5)
    
    second = provided.upa(500,20)
    third = provided.upa(500,80)
    fourth = provided.upa(500,320)
    
    print comp182.time_func(bfs, [first, 0]), "UPA(500,5)"
    print "Edges in graph:", edge_count(first)
    print comp182.time_func(bfs, [second, 0]),"UPA(500,20)"
    print "Edges in graph:", edge_count(second)
    print comp182.time_func(bfs, [third, 0]),"UPA(500,80)"
    print "Edges in graph:", edge_count(third)
    print comp182.time_func(bfs, [fourth, 0]),"UPA(500,320)"
    print "Edges in graph:", edge_count(fourth)
        
def node_count_function(graph):
    """Counts the number of nodes in a graph.

    Arguments:
    graph -- a dictioanry represention of a graph.

    Returns:
    An integer with the number of nodes in a graph.
    """
    return len(graph)

def edge_count(graph):
    """
    Returns the number of edges in a graph.

    Arguments:
    graph -- The given graph.

    Returns:
    The number of edges in the given graph.
    """
    edge_double_count = 0
    for nodeKey in graph.keys():
        edge_double_count = edge_double_count + len(graph[nodeKey])

    return edge_double_count / 2

def average_edge_per_node(graph):
    """Calls node count function and edge count to compute the average
    number of edges in a graph.

    Arguments:
    graph -- the given graph.

    Returns:
    An integer representing the average per node."""

    edges = edge_count(graph)
    print "Number of edges in graph:", edges
    nodes = node_count_function(graph)
    print "Number of nodes in graph:", nodes
    return "Average Edge Per node:", edges/nodes

networktopology = comp182.read_graph('rf7.repr')

def connectedcomponents(graph):
    """Returns a list of sets, where each set contains the nodes belonging to that connected component.
        The union of all of the sets in the list should contain all nodes in the graph.
     
    Arguments:
    graph -- The given graph.

     
    Returns:
    a list of sets, where each set contains the nodes belonging to that connected component.
    The union of all of the sets in the list should contain all nodes in the graph.

        """
    listofsets = []
    nodes = graph.keys()
    while len(nodes) > 0:
        randomnode = random.choice(nodes)
        (string,d,string1,p, string2,nds) = bfs(graph,randomnode)
        listofsets.append(nds)
        for node in list(nds):
            nodes.remove(node)
    return listofsets

def random_attack(graph):
    """Removes nodes randomly, one by one, and computes the size of the largest connected component for each resulting graph.

    Arguments:
    graph -- the given graph.

    Returns:
    A dictionary with keys for number of nodes removed and values for the size of the largest connected component for the graph."""

    random_attack_results = {} # empty dictionary places the number of nodes removed as the keys and largest connected component for the value
    nodes_randomly_attacked = set() #keeps track of the nodes that have been picked
    counter = 0 #keeps track of how many nodes removed
    nodestoremove = len(graph)
    
    while counter < nodestoremove:
        
        zero = 0
        for component in connectedcomponents(graph):
            value = 0 #places the size of the largest connected component
            
            for node in component:
                
                value += 1
            if value > zero:
                zero = value
        random_attack_results[counter] = zero
       
        nodechosen = random.choice(graph.keys()) #chooses the first
        nodes_randomly_attacked.add(nodechosen)
        graph.pop(nodechosen)
        for node in graph: #removes nodes from the graph
            if nodechosen in graph[node]:
                graph[node].remove(nodechosen)
        
        
        counter += 1   
        
        
    return random_attack_results

def find_popular_nodes(graph):
    """
    Find the set of all nodes in graph whose degree is higher than the
    average degree distribution in graph.

    Arguments:
    graph - an undirected graph

    Returns:
    Set of popular nodes in graph
    """
    popular = set()
    avgdeg = 2 * edge_count(graph) / node_count_function(graph)
    for node in graph:
        nodedeg = len(graph[node])
        if nodedeg > avgdeg:
            popular.add(node)
    return popular

def actual_size_degree(graph):
    """ computes the degree of popular nodes

    Agruments:
    graph -- a dictionary representation of a graph

    Returns:
    A list with nodes in order of highest degree."""

    setofnodes = find_popular_nodes(graph)
    print setofnodes
    emptylist = []
    for node in setofnodes:
        emptylist.append(len(graph[node]))
                      
    return emptylist

def targeted_attack(graph):
    """Removes nodes in decreasing order of degree, one by one, and compute
    the size of the largest connected component for each resulting graph.

    Arguments:
    graph -- an undirected graph.

    Returns:
    A dictionary number of nodes removed as the keys and
    size of largest connected components as the values."""

    targeted_attack_results = {} #stores the connected componments values
    counter = 0
    maxnodes = len(graph)
    while counter < maxnodes:
        zero = 0
        for component in connectedcomponents(graph): #finds the highest connected components
            value = 0
            
            for node in component:
                
                value += 1
            if value > zero:
                zero = value
        targeted_attack_results[counter] = zero
        highdegree = None
        for node in graph: #finds nodes of high degree and removes them
            founddegree = len(graph[node])
            if highdegree < founddegree:
                highdegree = founddegree
                removenode = node
                if highdegree == 0: #accounts for cases where there are no edges left
                    highdegree = 1
        #removes the node from the graph
        graph.pop(removenode)
        for edge in graph:
            if removenode in graph[edge]:
                graph[edge].remove(removenode)
        counter += 1

    return targeted_attack_results
    
def analyze_graphs():
    """ Runs all six experiments and plots the pictorial graphs.

    Arguments:
    None

    Returns:
    None"""
    #Analyzes the network graph and creates similar perimeters for Erdos and UPA graph
    nodes = node_count_function(networktopology)
    print "Nodes in Network graph:", nodes
    string1, edgesavg = average_edge_per_node(networktopology)
    edges = edge_count(networktopology)
    print "Averge Number of Edges:", edgesavg
    totaldegree = provided.total_degree(networktopology)
    print "Total degree of Network graph:", totaldegree
    probabilityerdos = float(edges)/(nodes*(nodes-1)/2.0)
    print "Probability P for Erdos:", probabilityerdos
    upa = provided.upa(nodes,edgesavg)

    #Saves the grahes to be modified.
    savedrandomgraph = comp182.copy_graph(upa)
    
    erdos = provided.erdos_renyi(nodes,probabilityerdos)
    
    savederdosgraph = comp182.copy_graph(erdos)
    
    savednetworkgraph = comp182.copy_graph(networktopology)
    #runs the different attacks on the various graphs
    randomgraphattack = random_attack(comp182.copy_graph(upa))
    print "First plot finished."
    randomgraphtargeted = targeted_attack(comp182.copy_graph(upa))
    print "Second plot finished."
    erdosgraphattack = random_attack(comp182.copy_graph(erdos))
    print "Third plot finished."
    erdosgraphtargeted = targeted_attack(comp182.copy_graph(erdos))
    print "Fourth plot finished."
    networkgraphattack = random_attack(comp182.copy_graph(savednetworkgraph))
    print "Fifth plot finished."
    networkgraphtargeted = targeted_attack(savednetworkgraph)
    print "Sixth plot finished."

    #formats the pictorial graph
    labelerdos = "Erdos({}, {})".format(nodes,probabilityerdos)
    labelupa = "UPA({},{})".format(nodes,edgesavg)
    labels = ["Network Graph"+"Targeted", "Network Graph"+"Random", labelerdos+"Targeted", labelerdos+"Random",labelupa+"Targeted", labelupa+"Random"]
    comp182.plot_lines([networkgraphtargeted, networkgraphattack, erdosgraphtargeted, erdosgraphattack, randomgraphtargeted, randomgraphattack],"Resiliency of Graphs' Attacks", "Nodes Removded", "Largest Connected-Component", labels, filename="Resiliency of Graphs' Attacks")
