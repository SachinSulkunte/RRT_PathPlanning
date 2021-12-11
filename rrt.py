import pygame
from rrtbase import RRTGraph
from rrtbase import RRTMap
import time

def main():
    dimensions =(600,1200)
    start=(50,50)
    goal=(1130,560)
    obsdim=30
    obsnum=150
    iteration=0
    t1=0

    pygame.init()
    map=RRTMap(start,goal,dimensions,obsdim,obsnum)
    graph=RRTGraph(start,goal,dimensions,obsdim,obsnum)

    obstacles=graph.makeobs()
    map.draw_map(obstacles)

    pygame.display.update()
    pygame.event.pump()
    time.sleep(1)
    t1=time.time()
    while (not graph.path_to_goal()):
        time.sleep(0.005)
        elapsed=time.time()-t1
        t1=time.time()
        #raise exception if timeout
        if elapsed > 10:
            print('timeout re-initiating the calculations')
            raise

        if iteration % 10 == 0:
            X, Y, Parent = graph.bias(goal)
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.node_radius*2, 0)
            pygame.draw.line(map.map, map.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                             map.edge_thickness)

        else:
            X, Y, Parent = graph.expand()
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.node_radius*2, 0)
            pygame.draw.line(map.map, map.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                             map.edge_thickness)

        if iteration % 5 == 0:
            pygame.display.update()
            #time.sleep(0.5)
            
        iteration += 1
    map.draw_path(graph.get_path_coords())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)



if __name__ == '__main__':
    result=False
    while not result:
        try:
            main()
            result=True
        except:
            result=False