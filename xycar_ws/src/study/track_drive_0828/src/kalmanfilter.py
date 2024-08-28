from pykalman import KalmanFilter
import numpy as np



class LaneKalmanFilter:
    state=np.array([0,0,0,0,0])
    vx=0
    dt=0.033
    
    transition_matrics=[

        [1,0,0,0,0],
        [1,0,0,0,0],
        [0,1,0,0,0],
        [0,0,1,0,0],
        [0,0,0,1 ,0],


    ]

    observation_matrics=[
        [1,0,0,0,0],
        [0,1,0,0,0],
        [0,0,1,0,0],
        [0,0,0,1,0],
        [0,0,0,0,1],


  ]
    
    uk = [-vx*dt]
    kf =KalmanFilter(transition_matrices= transition_matrics,observation_matrices=observation_matrics)
    


if __name__ =="__main__":

    meausrments  =[[20,22,23,24,25],[20,22,23,24,25],[20,22,23,24,25],[30,31,32,33,34]]
    pridictor= LaneKalmanFilter()
    (filtered_state_means, filtered_state_covariances) = pridictor.kf.filter(meausrments)
    print(filtered_state_means)
    # print(filtered_state_covariances)
    