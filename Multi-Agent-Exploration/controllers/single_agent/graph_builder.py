def outbound_expansion(graph, curr_vertex, sampled_point, sample_counter):
    new_vertex = "p" + str(sample_counter)
    graph.add_vertex(name=new_vertex, pos=sampled_point)
    graph.add_edge(curr_vertex, new_vertex)
    curr_vertex = new_vertex
    sample_counter = sample_counter + 1
    print(graph)
    return graph, curr_vertex, sample_counter

def inbound_consolidation(graph, position, sampled_point):
    pass