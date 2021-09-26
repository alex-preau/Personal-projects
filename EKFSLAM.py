import numpy as np
import matplotlib.pyplot as plt
from Renderer import Renderer
from Visualization import Visualization

class EKFSLAM(object):
    """A class for implementing EKF-based SLAM

        Attributes
        ----------
        mu :           The mean vector (numpy.array)
        Sigma :        The covariance matrix (numpy.array)
        R :            The process model covariance matrix (numpy.array)
        Q :            The measurement model covariance matrix (numpy.array)
        XGT :          Array of ground-truth poses (optional, may be None) (numpy.array)
        MGT :          Ground-truth map (optional, may be None)

        Methods
        -------
        prediction :   Perform the prediction step
        update :       Perform the measurement update step
        augmentState : Add a new landmark(s) to the state
        run :          Main EKF-SLAM loop
        render :       Render the filter
    """

    def __init__(self, mu, Sigma, R, Q, XGT = None, MGT = None):
        """Initialize the class

            Args
            ----------
            mu :           The initial mean vector (numpy.array)
            Sigma :        The initial covariance matrix (numpy.array)
            R :            The process model covariance matrix (numpy.array)
            Q :            The measurement model covariance matrix (numpy.array)
            XGT :          Array of ground-truth poses (optional, may be None) (numpy.array)
            MGT :          Ground-truth map (optional, may be None)
        """
        self.mu = np.zeros((3)) #26 monuments and 1 robot

        self.mu = mu



        self.Sigma = Sigma


        self.R = R
        self.Q = Q

        self.pi = 0 #to minimize error in update
        self.phi = np.zeros((2,2)) #holds minimzing k_denom
        self.H_min = np.zeros((2,2))
        self.delta = 0
        self.seen_landmark = False

        self.n = 0
        self.mu_sum = 0
        self.Sigma_sum = 0

        self.tmp_mu = self.mu
        self.tmp_Sigma = self.Sigma


        self.MU = mu
        self.VAR = np.diag(self.Sigma).reshape(3, 1)

        self.XGT = XGT
        self.MGT = MGT


        if (self.XGT is not None and self.MGT is not None):
            xmin = min(np.amin(XGT[1, :]) - 2, np.amin(MGT[1, :]) - 2)
            xmax = min(np.amax(XGT[1, :]) + 2, np.amax(MGT[1, :]) + 2)
            ymin = min(np.amin(XGT[2, :]) - 2, np.amin(MGT[2, :]) - 2)
            ymax = min(np.amax(XGT[2, :]) + 2, np.amax(MGT[2, :]) + 2)
            xLim = np.array((xmin, xmax))
            yLim = np.array((ymin, ymax))
        else:
            xLim = np.array((-8.0, 8.0))
            yLim = np.array((-8.0, 8.0))

        self.renderer = Renderer(xLim, yLim, 3, 'red', 'green')

        # Draws the ground-truth map
        if self.MGT is not None:
            self.renderer.drawMap(self.MGT)


        # You may find it useful to keep a dictionary that maps a feature ID
        # to the corresponding index in the mean vector and covariance matrix
        self.mapLUT = {}

    def prediction(self, u):
        """Perform the prediction step to determine the mean and covariance
           of the posterior belief given the current estimate for the mean
           and covariance, the control data, and the process model

            Args
            ----------
            u :  The forward distance and change in heading (numpy.array)
        """



    #    print(u)
        #print("map is len",(len(self.mapLUT)))
        F_x = self.F_x()

        correction = np.zeros(3)
        old_mu = self.mu[0:3] #old value should be sitting here
        #print("old mu is",old_mu)
        #print("u is",u)
        correction[0] =  (u[1]) * np.cos(old_mu[2])
        correction[1] =(u[1]) * np.sin(old_mu[2])
        correction[2] = (u[2])


        full_correction = np.dot((F_x),correction)

        len_sig = len(self.Sigma)

        F = np.dot(F_x,np.dot(self.F(u),np.transpose(F_x))) + np.identity(len_sig)
        G = self.G()

    #    print(F)
    #    print(self.Sigma)
        #sigma = F Sigma F
        # G R G
        #F

        new_sigma = np.dot(np.dot(F,self.Sigma),np.transpose(F))

        error_term =  np.dot(np.dot(G , self.R),np.transpose(G))

        error_term = np.dot(F_x,np.dot(error_term,np.transpose(F_x)))
    #    print(error_term)
        new_sigma = np.add(new_sigma ,error_term)
        #print(new_sigma)
        self.tmp_mu = np.add(self.mu,full_correction)
        #self.mu =self.tmp_mu
    #    self.tmp_mu[0] = tmp_mu
        self.tmp_Sigma = new_sigma
    #    self.tmp_Sigma[0:3,0:3] = new_sigma #only modify the ROBOT's position!!!

        # TODO: Your code goes here
        pass

    def F(self, u):
        #Jacobian F
        F = np.zeros((3,3))
        F[0][0] = 0#1 #the diagonal terms will be added in later through an I matrix
        F[1][1] = 0#1
        F[0][2] = -u[1] * np.sin(self.mu[2])#-u[1]/u[2] * np.cos(self.mu[2])+u[1]/u[2] * np.cos(self.mu[2] + u[2])#-u[1] * np.sin(self.mu[2])
        F[1][2] = u[1] * np.cos(self.mu[2])#-u[1]/u[2] * np.sin(self.mu[2])+u[1]/u[2] * np.sin(self.mu[2] + u[2])#u[1] * np.cos(self.mu[2])
        F[2][2] = 0#1
        return np.transpose(F)

    def G(self):
        G = np.zeros((3,2))
        G[0][0] = 2 * (self.mu[0])
        G[1][0] = 2 * (self.mu[1])
        #G[0][0] = np.cos(self.mu[2])
        #G[1][0] = np.sin(self.mu[2])
        G[2][1] = 1
        #print(G)
        return G

    def Gamma(self):
        Gamma = np.zeros((2,3))
        Gamma[0][0] = 1
        Gamma[1][1] = 1
        Gamma[0][2] = (np.sin(self.mu[2]) + np.cos(self.mu[2])) / ((np.sin(self.mu[2]) - np.cos(self.mu[2])) ** 2)
        Gamma[1][2] = (np.sin(self.mu[2]) - np.cos(self.mu[2])) / ((np.sin(self.mu[2]) + np.cos(self.mu[2])) ** 2)
        return Gamma

    def H(self,i):
        H = np.zeros((3 ,2))
        H[0][0] = (np.sin(self.mu[2]) - np.cos(self.mu[2]))
        H[1][1] = (-np.sin(self.mu[2]) - np.cos(self.mu[2]))
        H[2][0] =  (self.mu[i*3] - self.mu[0] )*(-np.sin(self.mu[2]) - np.cos(self.mu[2]))   #monument i+1_x
        H[2][1] =  (self.mu[i*3 + 1] - self.mu[1] )*(-np.sin(self.mu[2]) - np.cos(self.mu[2]))  #monument i+1_y

        return H

    def H_xv(self,i):
        H = np.zeros((3 ,2))
        denominator = ((self.mu[i*3] - self.mu[0])**2 + (self.mu[i*3+1] - self.mu[1])**2)
        H[0][0] = (self.mu[i*3] - self.mu[0])/np.sqrt(denominator)
        H[1][0] = (self.mu[i*3+1] - self.mu[1])/np.sqrt(denominator)
        H[0][1] = (self.mu[i*3+1] - self.mu[1])/(denominator)
        H[0][1] = (self.mu[i*3] - self.mu[0])/(denominator)
        H[2][1] = 1
        return H

    def F_x(self):
        F = np.zeros((3 + 2*len(self.mapLUT) ,3))

        F[0][0] = 1
        F[1][1] = 1
        F[2][2] = 1
        return F

    def F_update(self, id):
        F = np.zeros((5,3 + 2*len(self.mapLUT)))
        F[0][0] = 1
        F[1][1] = 1
        F[2][2] = 1
        key = self.mapLUT[id]
        F[3][key] = 1
        F[4][key +1] = 1
        return F

    def H_t(self, q, delta):
        H = np.zeros((2,5))
        H[0][0] = - np.sqrt(q) * delta[0]
        H[0][1] = - np.sqrt(q) * delta[1]
        H[0][3] =  np.sqrt(q) * delta[0]
        H[0][4] =  np.sqrt(q) * delta[1]
        H[1][0] = delta[1]
        H[1][1] = -delta[0]
        H[1][2] = -q
        H[1][3] = -delta[1]
        H[1][4] = delta[0]
        return H/q

    def H_correct(self,key,  mu):
        H = np.zeros((2,5))
        H[0][0] = -(np.cos(mu[2]) - np.sin(mu[2]))
        H[0][1] = 0
        H[0][2] =  -(np.cos(mu[2]) + np.sin(mu[2])) * (mu[key] - mu[0])
        H[0][3] =  (np.cos(mu[2]) - np.sin(mu[2]))
        H[1][1] = -(np.cos(mu[2]) + np.sin(mu[2]))
        H[1][2] = (np.cos(mu[2]) - np.sin(mu[2])) * (mu[key + 1] - mu[1])
        H[1][4] = (np.cos(mu[2]) + np.sin(mu[2]))
        return H







    def update(self, z, id):
        """Perform the measurement update step to compute the posterior
           belief given the predictive posterior (mean and covariance) and
           the measurement data

            Args
            ----------
            z :  The Cartesian coordinates of the landmark
                 in the robot's reference frame (numpy.array)
            id : The ID of the observed landmark (int)
        """
        if id not in self.mapLUT:
        #     r = np.sqrt(z[0]**2 + z[1]**2)
        #     bearing = np.arctan2(z[1],z[0])
        #
        #     if not self.mapLUT:
        #         key = 3
        #         self.mapLUT[id] = key
        #     else:
        #
        #         max_key = max(self.mapLUT.values())
        #         key = max_key + 2
        #         self.mapLUT[id] = key#self.mapLUT.max()
        #
        #     old_mu = self.tmp_mu
        #     old_mu_len = len( self.tmp_mu)
        #     self.mu = np.zeros((2*len(self.mapLUT) + 3))
        #     self.mu[0:old_mu_len] = old_mu
        # #    if id != 16:
        # #        return
        # #    print('\nNEW landmark',id)
        #         #if cov[2*j+3][2*j+3] >= 1e6 and cov[2*j+4][2*j+4] >= 1e6:
        #     # define landmark estimate as current measurement
        #     self.mu[key] = self.mu[0] + r*np.cos(bearing+self.mu[2])
        #     self.mu[key+1] = self.mu[1] + r*np.sin(bearing+self.mu[2])
        #
        #     old_sigma = self.tmp_Sigma
        #     s_len_old = len(old_sigma)
        # #    gamma = self.Gamma()
        #
        #     self.Sigma = np.zeros((s_len_old+2,s_len_old+2))
        #     self.Sigma[0:s_len_old,0:s_len_old] = old_sigma
        #     self.tmp_Sigma = self.Sigma
        #     self.tmp_mu = self.mu
        # #    print(self.Sigma)
            self.augmentState(z,id)

            #dont think we do antthing else either

        else:
            print("NEW CODE\n\n")
            mu = self.tmp_mu
            cov = self.tmp_Sigma
            key = self.mapLUT[id]
            N = len(mu)


            #cov = self.Sigma
            r = np.sqrt(z[0]**2 + z[1]**2)
            theta =  np.arctan2(z[1],z[0])
            # if landmark has not been observed before



            # if landmark is static

            # compute expected observation
            delta = np.array([mu[key]- mu[0], mu[key + 1] - mu[1]])
    #        print(delta)
        #    q = delta.T.dot(delta)

        #    sq = np.sqrt(q)

            #z_theta = np.arctan2(delta[1],delta[0])
            #z_hat = np.array([[sq], [z_theta-mu[2]]])
            z_x = (np.cos(mu[2]) - np.sin(mu[2])) * (mu[key] - mu[0])
            z_y = (np.cos(mu[2]) + np.sin(mu[2])) * (mu[key + 1] - mu[1])
            z_hat = np.array([[z_x], [z_y]])
    #        print(z_hat)
            # calculate Jacobian
            F = self.F_update(id)
        #    H = np.dot(self.H_t(q,delta), F)
            H = np.dot(self.H_correct(key,mu), F)

            #H_z = np.array([[-sq*delta[0], -sq*delta[1], 0, sq*delta[0], sq*delta[1]],
            #                [delta[1], -delta[0], -q, -delta[1], delta[0]]], dtype='float')
        #    H = 1/q*H_z.dot(F)

            # calculate Kalman gain
            K = cov.dot(H.T).dot(np.linalg.inv(H.dot(cov).dot(H.T)+self.Q))

            # calculate difference between expected and real observation
            z_dif = np.array([[z[0]],[z[1]]])-z_hat
            #z_dif = (z_dif + np.pi) % (2*np.pi) - np.pi

            # update state vector and covariance matrix
    #        print('orig mu',mu)
        #    mu = mu + K.dot(z_dif)/2
            if self.n  == 0:
                self.delta_mu = K.dot(z_dif)
                self.n += 1
            else:
                self.delta_mu  += K.dot(z_dif)
                self.n += 1
    #        print('new mu',mu[1])
            cov = (np.eye(N)-K.dot(H)/2).dot(cov)
            self.Sigma = cov


    def augmentState(self, z, id):
        """Augment the state vector to include the new landmark

            Args
            ----------
            z :  The Cartesian coordinates of the landmark
                 in the robot's reference frame (numpy.array)
            id : The ID of the observed landmark
        """
    #    print("ground truth map is\n",self.MGT)
    #    print("NEW")
    #    print("id is ",id)
    #    print(self.mu)
        if not self.mapLUT:
            key = 3
            self.mapLUT[id] = key
        else:

            max_key = max(self.mapLUT.values())
            key = max_key + 2
            self.mapLUT[id] = key#self.mapLUT.max()


        s_len_new = len(self.Sigma) + 2
        s_len_old = len(self.Sigma)



    #here is converted to range and bearing, a simpler measurement model that makes more sense here
    #    r = np.sqrt(z[0]**2 + z[1]**2)
    #    bearing = np.arctan2(z[1],z[0])


        old_mu = self.tmp_mu
        old_mu_len = len( self.tmp_mu)
        self.mu = np.zeros((2*len(self.mapLUT) + 3))

        self.mu[0:old_mu_len] = old_mu

        self.mu[key] = self.tmp_mu[0] - (np.sin(self.tmp_mu[2]) *z[1]) + np.cos(self.tmp_mu[2]) *z[0]#((np.cos(self.mu[2]) + np.sin(self.mu[2]) ) ** -1) * z[0] + self.mu[0]
        self.mu[key + 1] =  self.tmp_mu[1] + (np.sin(self.tmp_mu[2]) *z[0] + np.cos(self.tmp_mu[2]) *z[1])#((np.cos(self.mu[2]) - np.sin(self.mu[2]) ) ** -1) * z[1] + self.mu[1]
        self.tmp_mu = self.mu
    #    print("new landmark pos is ",self.mu[key*2 + 1],self.mu[key*2 + 2])

        old_sigma = self.tmp_Sigma
        gamma = self.Gamma()

        self.Sigma = np.zeros((s_len_new,s_len_new))
        self.Sigma[0:s_len_old,0:s_len_old] = old_sigma



        #robot new landmark
        self.Sigma[s_len_old:s_len_new,0:3] = np.dot(gamma,old_sigma[0:3,0:3])
        self.Sigma[0:3,s_len_old:s_len_new] = np.dot(old_sigma[0:3,0:3],np.transpose(gamma))

        #landmark itself
        self.Sigma[s_len_old:s_len_new,s_len_old:s_len_new] = np.dot(gamma,np.dot(old_sigma[0:3,0:3],np.transpose(gamma))) + self.Q
        #print("landmark sigma\n",np.dot(gamma,np.dot(old_sigma[0:3,0:3],np.transpose(gamma))) + self.Q)
        #print(np.dot(gamma,np.dot(old_sigma[0:3,0:3],np.transpose(gamma))))

        #landmark other landmark

        num_other_landmarks = int((s_len_old - 3) / 2)

        for i in range(1,num_other_landmarks+1):

            landmark_sigma = old_sigma[s_len_old-(2*i):s_len_old-(2 * (i-1)),0:3]

            new_sigma = np.dot((gamma),np.transpose(landmark_sigma))

            self.Sigma[s_len_old:s_len_new,(3+i*2):(3+i*2+2)] = new_sigma
            self.Sigma[(3+i*2):(3+i*2+2),s_len_old:s_len_new] = np.transpose(new_sigma)

        self.tmp_Sigma = self.Sigma
        self.tmp_mu = self.mu
    #    print("whole sigma \n",self.Sigma)


        # TODO: Your code goes here
        pass

    def angleWrap(self, theta):
        """Ensure that a given angle is in the interval (-pi, pi)."""
        while theta < -np.pi:
            theta = theta + 2*np.pi

        while theta > np.pi:
            theta = theta - 2*np.pi

        return theta

    def run(self, U, Z):
        """The main loop of EKF-based SLAM

            Args
            ----------
            U :   Array of control inputs, one column per time step (numpy.array)
            Z :   Array of landmark observations in which each column
                  [t; id; x; y] denotes a separate measurement and is
                  represented by the time step (t), feature id (id),
                  and the observed (x, y) position relative to the robot
        """
        # TODO: Your code goes here
        for t in range(np.size(U, 1)):
            self.pi = -1
            self.n  = 0
            self.Sigma = self.tmp_Sigma
            self.mu = self.tmp_mu
            self.prediction(U[:, t])
            self.delta_mu = 0
        #     self.update(Z[:, t])

        #    self.MU = np.column_stack((self.MU, self.mu))
        #    self.VAR = np.column_stack((self.VAR, np.diag(self.Sigma)))
            Zt = np.array(np.zeros((4,0)))

            self.seen_landmark = False
            unseen_list_id = []
            unseen_list_value = []
            seen_list_id = []
            seen_list_value = []
            for enum, (i) in enumerate(Z[0]):
                continue
                i = int(i)



                if (i) == t:
                #    print(Z[:,enum])
                    if int(Z[1, enum]) not in self.mapLUT:
                #        print("adding",int(Z[1, enum]))
                        unseen_list_id.append(int(Z[1, enum]) )
                        unseen_list_value.append(Z[2:4, enum])
                    elif int(Z[1, enum])  in self.mapLUT:
                        seen_list_id.append(int(Z[1, enum]) )
                        seen_list_value.append(Z[2:4, enum])
                    if Zt.shape[1] == 0:

                        Zt = np.zeros((4,1))

                        Zt = np.hstack((Zt,np.array(Z[:,enum]).reshape((4,1))))

                        Zt = np.delete(Zt,0,1)
                    else:
                        z_new = np.zeros((4,1))


                        Zt = np.hstack((Zt,np.array(Z[:,enum]).reshape((4,1))))

            if self.n != 0:
            #    print(self.delta_mu/(235*self.n))
                #SMALL WAS 998
                sum = abs(np.sum(self.delta_mu))
                #print(sum)
                print(self.delta_mu/(sum*self.n))
                #large is 99.99 * mu_sum
                denominator_small_error = 998 * self.n
                denominator_large_error = 99.99 * sum * self.n
                self.mu = (self.tmp_mu + self.delta_mu/(denominator_large_error))[0]#(99.99*sum*self.n))[0]
            else:
                self.mu = self.tmp_mu
                self.sigma = self.tmp_Sigma

            self.tmp_mu = self.mu
            self.tmp_Sigma = self.Sigma

            self.renderer.render(self.mu, self.Sigma, self.XGT[1:4, t], Zt, self.mapLUT)
            self.MU = np.column_stack((self.MU, self.mu[0:3]))
            self.VAR = np.column_stack((self.VAR, np.diag(self.Sigma[0:3,0:3])))

        self.renderer.drawTrajectory(self.MU[0:3, :], self.XGT[1:4, :])
        self.renderer.plotError(self.MU[0:3, :], self.XGT[1:4, :], self.VAR[0:3, :])

        plt.ioff()
        plt.show()
        pass

        # You may want to call the visualization function between filter steps where
        #       self.XGT[1:4, t] is the column of XGT containing the pose the current iteration
        #       Zt are the columns in Z for the current iteration
        #       self.mapLUT is a dictionary where the landmark IDs are the keys
        #                   and the index in mu is the value
        #
        # self.renderer.render(self.mu, self.Sigma, self.XGT[1:4, t], Zt, self.mapLUT)
