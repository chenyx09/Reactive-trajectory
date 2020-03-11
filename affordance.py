import numpy as np
lane_width = 3.6


def calc_obs(veh_set,N_lane=6):
    obs = list()
    for i in range(0,N_lane):
        obs.append([])
    for i in range(0,len(veh_set)):

        lane_ID = int(round((veh_set[i].state[1]-1.8)/3.6))
        if lane_ID>N_lane-1:
            lane_ID = N_lane-1
        elif lane_ID<0:
            lane_ID = 0

        ub = veh_set[i].state[0]
        lb = ub - veh_set[i].v_length
        left_X = veh_set[i].state[1]-veh_set[i].v_width/2
        right_X = veh_set[i].state[1]+veh_set[i].v_width/2
        v_vel =veh_set[i].state[2]
        obs[lane_ID].append([lb, ub, left_X, right_X, v_vel])

    for i in range(0,N_lane):
        obs[i]=np.array(obs[i])
        if obs[i].any():
            # print(obs[i][0])

            # print(obs[i].shape)
            obs[i]=obs[i][obs[i][:,0].argsort(),]
    return obs
def calc_affordance(veh_set,N_lane=6):

    veh_affordance=[None]*len(veh_set)
    M = 50
    obs = calc_obs(veh_set,N_lane)

    for i in range(0,len(veh_set)):
        lane_ID = int(round((veh_set[i].state[1]-1.8)/3.6))
        if lane_ID>N_lane-1:
            lane_ID = N_lane-1
        elif lane_ID<0:
            lane_ID = 0

        X = veh_set[i].state[1]
        Y = veh_set[i].state[0]
        L = veh_set[i].v_length
        W = veh_set[i].v_width
        v0 = veh_set[i].state[2]
        dis2cen = X-(lane_ID*lane_width+lane_width/2)
        min_free_space = max(2*L,6);



        if lane_ID==0:
            left_front_Y = 0
            left_rear_Y = 0
            left_front_vel = 0
            left_rear_vel = 0
            left_front_X = dis2cen-W/2+1.8
            left_rear_X  = dis2cen-W/2+1.8
            left_fwd_Y = M
            left_fwd_vel = 0
            left_front_L = 0
            left_rear_L = 0
            if obs[lane_ID].any():
                obs1=obs[lane_ID][obs[lane_ID][:,1]>Y]
                obs2=obs[lane_ID][obs[lane_ID][:,1]<Y]
                # if i==0:
                #     print("obs1=",obs1)
                #     print("obs2=",obs2)
            else:
                obs1=np.array([])
                obs2=np.array([])

            if obs1.any():
                fwd_dis = obs1[0,0]-Y
                fwd_vel = obs1[0,4]-v0
            else:
                fwd_dis = M
                fwd_vel = 0

            if obs2.any():
                rear_dis = Y-L-obs2[-1,1]
                rear_vel = obs2[-1,4]-v0
            else:
                rear_dis = M
                rear_vel = 0
            if len(obs)>1 and obs[lane_ID+1].any():
                obs1=obs[lane_ID+1][obs[lane_ID+1][:,1]>Y-L/2]
                obs2=obs[lane_ID+1][obs[lane_ID+1][:,1]<Y-L/2]

            else:
                obs1=np.array([])
                obs2=np.array([])

            right_front_X = 3.6

            if obs1.any():
                right_front_X = min(right_front_X,obs1[0,2]-X-W/2);
                right_front_vel = obs1[0,4]-v0
                right_front_Y = obs1[0,0]-Y
                right_front_L = obs1[0,1]-obs1[0,0]
                j=0
                while j<obs1.shape[0]-1:
                    if obs1[j+1,0]-obs1[j,1]>min_free_space:
                        break
                    else:
                        j=j+1

                    if obs1[j,1]-Y>M:
                        break

                right_fwd_Y = obs1[j,1]-Y-L
                right_fwd_vel = obs1[j,4]-v0
            else:
                right_front_Y = M
                right_front_vel = 0
                right_fwd_Y = -M
                right_fwd_vel = 0
                right_front_L = 0

            right_rear_X  = 3.6
            if obs2.any():
                right_rear_X = min(right_rear_X,obs2[-1,2]-X-W/2)
                right_rear_Y = Y-obs2[-1,1]
                right_rear_vel = obs2[-1,4]-v0
                right_rear_L = obs2[-1,1]-obs2[-1,0]
            else:
                right_rear_Y = M
                right_rear_vel = 0
                right_rear_L = 0

        elif lane_ID>1 and lane_ID<N_lane-1:

            if obs[lane_ID].any():
                obs1=obs[lane_ID][obs[lane_ID][:,1]>Y]
                obs2=obs[lane_ID][obs[lane_ID][:,1]<Y]
            else:
                obs1=np.array([])
                obs2=np.array([])

            if obs1.any():
                fwd_dis = obs1[0,0]-Y
                fwd_vel = obs1[0,4]-v0
            else:
                fwd_dis = M
                fwd_vel = 0


            if obs2.any():
                rear_dis = Y-L-obs2[-1,1]
                rear_vel = obs2[-1,4]-v0
            else:
                rear_dis = M
                rear_vel = 0

            if obs[lane_ID+1].any():
                obs1=obs[lane_ID+1][obs[lane_ID+1][:,1]>Y-L/2]
                obs2=obs[lane_ID+1][obs[lane_ID+1][:,1]<Y-L/2]
            else:
                obs1=np.array([])
                obs2=np.array([])

            right_front_X = 3.6

            if obs1.any():
                right_front_X = min(right_front_X,obs1[0,2]-X-W/2)
                right_front_vel = obs1[0,4]-v0
                right_front_Y = obs1[0,0]-Y
                right_front_L = obs1[0,1]-obs1[0,0]
                j=0
                while j<obs1.shape[0]-1:
                    if obs1[j+1,0]-obs1[j,1]>min_free_space:
                        break
                    else:
                        j=j+1
                    if obs1[j,1]-Y>M:
                        break
                right_fwd_Y = obs1[j,1]-Y-L
                right_fwd_vel = obs1[j,4]-v0
            else:
                right_front_Y = M
                right_front_vel = 0
                right_fwd_Y = -M
                right_fwd_vel = 0
                right_front_L = 0
            right_rear_X  = 3.6
            if obs2.any():
                right_rear_X = min(right_rear_X,obs2[-1,2]-X-W/2)
                right_rear_Y = Y-obs2[-1,1]
                right_rear_vel = obs2[-1,4]-v0;
                right_rear_L = obs2[-1,1]-obs2[-1,0]
            else:
                right_rear_Y = M
                right_rear_vel = 0
                right_rear_L = 0

            if obs[lane_ID-1].any():
                obs1=obs[lane_ID-1][obs[lane_ID-1][:,1]>Y-L/2]
                obs2=obs[lane_ID-1][obs[lane_ID-1][:,1]<Y-L/2]
            else:
                obs1=np.array([])
                obs2=np.array([])

            left_front_X = 3.6

            if obs1.any():
                left_front_X = min(left_front_X,X-obs1[0,3]-W/2)
                left_front_vel = obs1[0,4]-v0
                left_front_Y = obs1[0,0]-Y
                left_front_L = obs1[0,1]-obs1[0,0]
                j=0
                while j<obs1.shape[0]-1:
                    if obs1[j+1,0]-obs1[j,1]>min_free_space:
                        break
                    else:
                        j=j+1

                    if obs1[j,1]-Y>M:
                        break

                left_fwd_Y = obs1[j,1]-Y-L
                left_fwd_vel = obs1[j,4]-v0
            else:
                left_front_Y = M
                left_front_vel = 0
                left_fwd_Y = -M
                left_fwd_vel = 0
                left_front_L = 0

            left_rear_X  = 3.6
            if obs2.any():
                left_rear_X = min(left_rear_X,X-obs2[-1,3]-W/2);
                left_rear_Y = Y-obs2[-1,1];
                left_rear_vel = obs2[-1,4]-v0;
                left_rear_L = obs2[-1,1]-obs2[-1,0];
            else:
                left_rear_Y = M
                left_rear_vel = 0
                left_rear_L = 0

        elif lane_ID==N_lane-1:
            right_front_Y = 0
            right_rear_Y = 0
            right_front_X = 1.8-dis2cen-W/2
            right_rear_X  = 1.8-dis2cen-W/2
            right_front_vel = 0
            right_rear_vel = 0
            right_fwd_Y = M
            right_fwd_vel = 0
            right_front_L = 0
            right_rear_L = 0
            if obs[lane_ID].any():
                obs1=obs[lane_ID][obs[lane_ID][:,1]>Y]
                obs2=obs[lane_ID][obs[lane_ID][:,1]<Y]
            else:
                obs1=np.array([])
                obs2=np.array([])


            if obs1.any():
                fwd_dis = obs1[0,0]-Y
                fwd_vel = obs1[0,4]-v0
            else:
                fwd_dis = M
                fwd_vel = 0

            if obs2.any():
                rear_dis = Y-L-obs2[-1,1]
                rear_vel = obs2[-1,4]-v0
            else:
                rear_dis = M
                rear_vel = 0


            if obs[lane_ID-1].any():
                obs1=obs[lane_ID-1][obs[lane_ID-1][:,1]>Y-L/2]
                obs2=obs[lane_ID-1][obs[lane_ID-1][:,1]<Y-L/2]
            else:
                obs1=np.array([])
                obs2=np.array([])

            left_front_X = 3.6

            if obs1.any():
                left_front_X = min(left_front_X,X-obs1[0,3]-W/2)
                left_front_vel = obs1[0,4]-v0
                left_front_Y = obs1[0,0]-Y
                left_front_L = obs1[0,1]-obs1[0,0]
                j=0
                while j<obs1.shape[0]-1:
                    if obs1[j+1,0]-obs1[j,1]>min_free_space:
                        break
                    else:
                        j=j+1
                    if obs1[j,1]-Y>M:
                        break

                left_fwd_Y = obs1[j,1]-Y-L
                left_fwd_vel = obs1[j,4]-v0
            else:
                left_front_Y = M
                left_front_vel = 0
                left_fwd_Y = -M
                left_fwd_vel = 0
                left_front_L = 0

            left_rear_X  = 3.6
            if obs2.any():
                left_rear_X = min(left_rear_X,X-obs2[-1,3]-W/2)
                left_rear_Y = Y-obs2[-1,1]
                left_rear_vel = obs2[-1,4]-v0
                left_rear_L = obs2[-1,1]-obs2[-1,0]
            else:
                left_rear_Y = M
                left_rear_vel = 0
                left_rear_L = 0


        veh_affordance[i] = [lane_ID, v0, dis2cen, fwd_dis, fwd_vel, rear_dis, rear_vel, left_front_Y,
                            left_front_X, left_front_vel, left_rear_Y, left_rear_X, left_rear_vel,
                             right_front_Y, right_front_X, right_front_vel, right_rear_Y, right_rear_X,
                             right_rear_vel, left_fwd_Y, left_fwd_vel, right_fwd_Y, right_fwd_vel,
                             left_front_L, left_rear_L, right_front_L, right_rear_L, L]

    veh_affordance = np.array(veh_affordance)

    veh_affordance[:,[3,5,7,10,13,16,19,21]]=np.clip(veh_affordance[:,[3,5,7,10,13,16,19,21]],-M,M)
    return veh_affordance
