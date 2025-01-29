import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
import matplotlib
from Frame import Frame

matplotlib.use('Qt5Agg')

#Robot parameters
l1 = 478
l2 = 425
l3 = 425
endEffector = 100
#Total 1428

def simplify_angle(angle):
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle

def simplify_angles(angles):
    for i in range(angles.shape[0]):
        angles[i] = simplify_angle(angles[i])
    return angles

class Staubli(object):
    def __init__(self, dh_params, plot_xlim=[-0.5, 0.5], plot_ylim=[-0.5, 0.5], plot_zlim=[0.0, 1.0], step_size=5e-1, max_iter=300, final_loss=1e-4):
        self.params = dh_params[:, 0:3]
        self.initial_offset = dh_params[:, 3]
        self.axis_values = np.zeros(dh_params[:, 3].shape, dtype=np.float64)
        # is_reachable_inverse must be set everytime when inverse kinematics is performed
        self.is_reachable_inverse = True
        # plot related
        self.plot_xlim = plot_xlim
        self.plot_ylim = plot_ylim
        self.plot_zlim = plot_zlim
        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(111, projection="3d")

        self.step_size = step_size
        self.max_iter = max_iter
        self.final_loss = final_loss

    @property
    def dh_params(self):
        return np.hstack((self.params, (self.axis_values + self.initial_offset).reshape([self.num_axis, 1])))

        # transformation between axes

    @property
    def ts(self):
        dh = self.dh_params
        ts = []
        for i in range(self.num_axis):
            ts.append(Frame.from_dh(dh[i]))
        return ts

    # base to end transformation
    @property
    def axis_frames(self):
        ts = self.ts
        fs = []
        f = Frame.i_4_4()
        for i in range(self.num_axis):
            f = f * ts[i]
            fs.append(f)
        return fs

    @property
    def end_frame(self):
        return self.axis_frames[-1]

    @property
    def jacobian(self):
        axis_fs = self.axis_frames
        jac = np.zeros([6, self.num_axis])
        jac[0:3, 0] = np.cross(np.array([0., 0., 1.]), axis_fs[-1].t_3_1.reshape([3, ]))
        jac[3:6, 0] = np.array([0., 0., 1.])
        for i in range(1, self.num_axis):
            jac[0:3, i] = np.cross(axis_fs[i - 1].z_3_1.reshape([3, ]),
                                    (axis_fs[-1].t_3_1 - axis_fs[i - 1].t_3_1).reshape([3, ]))
            jac[3:6, i] = axis_fs[i - 1].z_3_1.reshape([3, ])
        return jac

    def inverse(self, end_frame):
        last_dx = np.zeros([6, 1])
        for _ in range(self.max_iter):
            jac = np.linalg.pinv(self.jacobian)
            end = self.end_frame
            dx = np.zeros([6, 1])
            dx[0:3, 0] = (end_frame.t_3_1 - end.t_3_1).reshape([3, ])
            diff = end.inv * end_frame
            dx[3:6, 0] = end.r_3_3.dot(diff.r_3.reshape([3, 1])).reshape([3, ])
            if np.linalg.norm(dx, ord=2) < self.final_loss or np.linalg.norm(dx - last_dx,
                                                                             ord=2) < 0.1 * self.final_loss:
                self.axis_values = simplify_angles(self.axis_values)
                self.is_reachable_inverse = True
                return self.axis_values
            dq = self.step_size * jac.dot(dx)
            self.forward(self.axis_values + dq.reshape([self.num_axis, ]))
            last_dx = dx
        self.is_reachable_inverse = False
        raise Exception("Pose cannot be reached!")


    def draw(self):
        self.ax.clear()
        self.plot_settings()
        # plot the arm
        x, y, z = [0.], [0.], [0.]
        axis_frames = self.axis_frames
        for i in range(self.num_axis):
            x.append(axis_frames[i].t_3_1[0, 0])
            y.append(axis_frames[i].t_3_1[1, 0])
            z.append(axis_frames[i].t_3_1[2, 0])
        self.ax.plot_wireframe(x, y, np.array([z]))
        self.ax.scatter(x[1:], y[1:], z[1:], c="red", marker="o")
        # plot axes using cylinders
        cy_radius = np.amax(self.params[:, 0:2]) * 0.05
        cy_len = cy_radius * 4.
        cy_div = 4 + 1
        theta = np.linspace(0, 2 * np.pi, cy_div)
        cx = np.array([cy_radius * np.cos(theta)])
        cz = np.array([-0.5 * cy_len, 0.5 * cy_len])
        cx, cz = np.meshgrid(cx, cz)
        cy = np.array([cy_radius * np.sin(theta)] * 2)
        points = np.zeros([3, cy_div * 2])
        points[0] = cx.flatten()
        points[1] = cy.flatten()
        points[2] = cz.flatten()
        self.ax.plot_surface(points[0].reshape(2, cy_div), points[1].reshape(2, cy_div), points[2].reshape(2, cy_div),
                             color="pink", rstride=1, cstride=1, linewidth=0, alpha=0.4)
        for i in range(self.num_axis - 1):
            f = axis_frames[i]
            points_f = f.r_3_3.dot(points) + f.t_3_1
            self.ax.plot_surface(points_f[0].reshape(2, cy_div), points_f[1].reshape(2, cy_div),
                                 points_f[2].reshape(2, cy_div)
                                 , color="pink", rstride=1, cstride=1, linewidth=0, alpha=0.4)
        # plot the end frame
        f = axis_frames[-1].t_4_4
        self.ax.plot_wireframe(np.array([f[0, 3], f[0, 3] + 0.2 * f[0, 0]]),
                               np.array([f[1, 3], f[1, 3] + 0.2 * f[1, 0]]),
                               np.array([[f[2, 3], f[2, 3] + 0.2 * f[2, 0]]]), color="red")
        self.ax.plot_wireframe(np.array([f[0, 3], f[0, 3] + 0.2 * f[0, 1]]),
                               np.array([f[1, 3], f[1, 3] + 0.2 * f[1, 1]]),
                               np.array([[f[2, 3], f[2, 3] + 0.2 * f[2, 1]]]), color="green")
        self.ax.plot_wireframe(np.array([f[0, 3], f[0, 3] + 0.2 * f[0, 2]]),
                               np.array([f[1, 3], f[1, 3] + 0.2 * f[1, 2]]),
                               np.array([[f[2, 3], f[2, 3] + 0.2 * f[2, 2]]]), color="blue")

    @property
    def num_axis(self):
        return self.initial_offset.shape[0]


    def forward(self, theta_x):
        self.axis_values = theta_x
        return self.end_frame

    def plot_settings(self):
        self.ax.set_xlim(self.plot_xlim)
        self.ax.set_ylim(self.plot_ylim)
        self.ax.set_zlim(self.plot_zlim)
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_zlabel("z")

    def show(self):
        self.draw()
        plt.show()

#Question 1
#Affichage de la photo du robot
#TODO Ajouter une photo avec les informations sur les joints
def show_image(image_path, title='Image'):
    image = plt.imread(image_path)
    plt.imshow(image)
    plt.axis('off')
    plt.title(title)
    plt.show()

#Question 2
#Representation du robot en home position


def go_home(robot):
    theta = np.array([0, 0, 0, 0, 0, 0])
    robot.forward(theta)
    robot.show()
#Question 3
#Parametres de DH

def DH_parameters():
    dh_matrix=[]
    return dh_matrix


#Question 4
#Cinématique directe

def forward_ik(robot):
    '''Fonction qui demande à l'utilisateur de rentrer un angle pour chaque membre et qui deplace le robot vers cette position'''
    #test value for theta
    theta = np.array([0., 0., -0.25 * pi, 0., 0., 0.])
    '''
    q1 = int(input("q1: "))
    q2 = int(input("q2: "))
    q3 = int(input("q3: "))
    q4 = int(input("q4: "))
    q5 = int(input("q5: "))
    q6 = int(input("q6: "))
    theta = np.array([q1, q2, q3, q4, q5, q6])'''

    robot.forward(theta)
    robot.show()


#Question 5
#Cinématique inverse

def inverse_ik(robot):
    xyz = np.array([[0.28127], [0.], [1.13182]]) #Coordonnees
    abc = np.array([0.5 * pi, 0., pi])           # Roll, Pitch, Yaw
    end = Frame.from_euler_3(abc, xyz)
    robot.inverse(end)

    print("inverse is successful: {0}".format(robot.is_reachable_inverse))
    print("axis values: \n{0}".format(robot.axis_values))
    robot.show()

    # example of unsuccessful inverse kinematics
    xyz = np.array([[2.2], [0.], [1.9]])
    end = Frame.from_euler_3(abc, xyz)
    robot.inverse(end)

    print("inverse is successful: {0}".format(robot.is_reachable_inverse))

def animate(robot):
    frames = [Frame.from_euler_3(np.array([0.5 * pi, 0., pi]), np.array([[0.28127], [0.], [1.13182]])),
              Frame.from_euler_3(np.array([0.25 * pi, 0., 0.75 * pi]), np.array([[0.48127], [0.], [1.13182]])),
              Frame.from_euler_3(np.array([0.5 * pi, 0., pi]), np.array([[0.48127], [0.], [0.63182]]))]
    time_points = np.array([0., 6., 10.])
    trajectory = RobotTrajectory(robot, frames, time_points)
    trajectory.show(motion="p2p")


def main():
    #Parametres sous la forme d | a | alpha | theta
    dh_params = np.array([[0.478, 0.050, -pi*0.5, 0      ],   #1
                          [0    , 0.425, 0      , -pi*0.5],   #2
                          [0.050, 0    , pi*0.5 , pi*0.5 ],   #3
                          [0.425, 0    , -pi*0.5, 0      ],   #4
                          [0    , 0    , pi/2   , 0      ],   #5
                          [0.100, 0    , 0      , 0      ]])  #6
    robot = Staubli(dh_params)

    np.set_printoptions(precision=3, suppress=True)

    print("Choose option \n"
          "1 : Photo\n"
          "2 : Go to home position\n"
          "3 : Parametres de denavit-hartenberg\n"
          "4 : Forward Kinematics\n"
          "5 : Inverse Kinematerg\n"
          "6 : Animation\n"
          "7 : Quit")
    user_choice = input()

    if user_choice == "1":
        show_image("robot.png")

    elif user_choice == "2":
        go_home(robot)

    elif user_choice == "3":
        print(dh_params)

    elif user_choice == "4":
        forward_ik(robot)

    elif user_choice == "5":
        inverse_ik(robot)

    elif user_choice == "6":
        animate(robot)

if __name__ == '__main__':
    main()