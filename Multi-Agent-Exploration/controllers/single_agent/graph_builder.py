'''
Take the backward reachable graph and expand by using the newly sampled point.
'''
def outbound_expansion(g_f, curr_vertex, sampled_point, sample_counter):
    new_vertex = "p" + str(sample_counter)
    g_f.add_vertex(name=new_vertex, pos=sampled_point)
    g_f.add_edge(curr_vertex, new_vertex)
    curr_vertex = new_vertex
    sample_counter = sample_counter + 1
    print(g_f)
    return g_f, curr_vertex, sample_counter

def inbound_consolidation(curr_pos, g_f, g_b, curr_vertex, mvController):
    backward_path = g_f.get_shortest_paths(curr_vertex, "home", mode="in", output='vpath')
    print("Path back home: ", backward_path)

    for i in backward_path[0]:
        if (i==0):
            backward_point = g_f.vs.find(name="home")["pos"]
            print("Move to destination", backward_point)
            mvController.moveToDestination(backward_point, curr_pos)
        else:
            backward_point = g_f.vs.find(name="p" + str(i))["pos"]
            print("Move to destination", backward_point)
            mvController.moveToDestination(backward_point, curr_pos)

    curr_vertex = "home"

    return g_f, g_b, curr_vertex