import numpy as np
import pypoman as pm
import matplotlib.pyplot as plt


def plotACS(B_nom, ineq, ineq_fault, ineq_f_pseudo, mass, radius):
    
    # calculate and plot ACS

    x = []
    y = []
    x_fault = []
    y_fault = []
    x_pseudo = []
    y_pseudo = []

    # yaw and thrust
    proj = (B_nom[[2, 3],:], np.zeros(2))  # 2D projection of polytope y = E * x + f, select yaw and thrust
    eq = (B_nom[[0, 1],:], np.zeros(2))  # equality constraints C * x = d, roll and pitch moment = zero

    vertices = np.array(pm.project_polytope(proj, ineq, eq, method='bretl'))
    x.append(list(vertices[:,0]))  # list of x coordinates of vertices for plot
    y.append(list(vertices[:,1]))  # list of y coordinates of vertices for plot
    #print(vertices)

    #proj_fault = (B_fault[[2, 3],:], np.zeros(2))  # 2D projection of polytope y = E * x + f, select yaw and thrust
    #eq_fault = (B_fault[[0, 1],:], np.zeros(2))  # equality constraints C * x = d, roll and pitch moment = zero

    vertices_fault = np.array(pm.project_polytope(proj, ineq_fault, eq, method='bretl'))
    #vertices_fault = np.array(pm.project_polytope(proj_fault, ineq_fault, eq_fault, method='bretl'))
    x_fault.append(list(vertices_fault[:,0]))
    y_fault.append(list(vertices_fault[:,1]))
    #print(vertices_fault)

    proj = (np.array([[0,0,1,0],[0,0,0,1]]), np.zeros(2))  # 2D projection, yaw and thrust
    eq = (np.array([[1,0,0,0],[0,1,0,0]]), np.zeros(2))    # equality constraint for roll and pitch
    vertices_pseudo = np.array(pm.project_polytope(proj, ineq_f_pseudo, eq, method='bretl'))
    x_pseudo.append(list(vertices_pseudo[:,0]))
    y_pseudo.append(list(vertices_pseudo[:,1]))
    #print(vertices_pseudo)

    # pitch and thrust
    proj = (B_nom[[1, 3],:], np.zeros(2))  # 2D projection of polytope y = E * x + f, select pitch and thrust
    eq = (B_nom[[0, 2],:], np.zeros(2))  # equality constraints C * x = d, roll and yaw moment = zero

    vertices = np.array(pm.project_polytope(proj, ineq, eq, method='bretl'))
    x.append(list(vertices[:,0]))  # list of x coordinates of vertices for plot
    y.append(list(vertices[:,1]))  # list of y coordinates of vertices for plot

    vertices_fault = np.array(pm.project_polytope(proj, ineq_fault, eq, method='bretl'))
    x_fault.append(list(vertices_fault[:,0]))
    y_fault.append(list(vertices_fault[:,1]))

    proj = (np.array([[0,1,0,0],[0,0,0,1]]), np.zeros(2))  # 2D projection, pitch and thrust
    eq = (np.array([[1,0,0,0],[0,0,1,0]]), np.zeros(2))    # equality constraint for roll and yaw
    vertices_pseudo = np.array(pm.project_polytope(proj, ineq_f_pseudo, eq, method='bretl'))
    x_pseudo.append(list(vertices_pseudo[:,0]))
    y_pseudo.append(list(vertices_pseudo[:,1]))

    # roll and thrust
    proj = (B_nom[[0, 3],:], np.zeros(2))  # 2D projection of polytope y = E * x + f, select roll and thrust
    eq = (B_nom[[1, 2],:], np.zeros(2))  # equality constraints C * x = d, pitch and yaw moment = zero

    vertices = np.array(pm.project_polytope(proj, ineq, eq, method='bretl'))
    x.append(list(vertices[:,0]))  # list of x coordinates of vertices for plot
    y.append(list(vertices[:,1]))  # list of y coordinates of vertices for plot

    vertices_fault = np.array(pm.project_polytope(proj, ineq_fault, eq, method='bretl'))
    x_fault.append(list(vertices_fault[:,0]))
    y_fault.append(list(vertices_fault[:,1]))

    proj = (np.array([[1,0,0,0],[0,0,0,1]]), np.zeros(2))  # 2D projection, roll and thrust
    eq = (np.array([[0,0,1,0],[0,1,0,0]]), np.zeros(2))    # equality constraint for pitch and yaw
    vertices_pseudo = np.array(pm.project_polytope(proj, ineq_f_pseudo, eq, method='bretl'))
    x_pseudo.append(list(vertices_pseudo[:,0]))
    y_pseudo.append(list(vertices_pseudo[:,1]))

    # roll and pitch
    proj = (B_nom[[0, 1],:], np.zeros(2))  # 2D projection of polytope y = E * x + f, select roll and pitch axis
    eq = (B_nom[[2, 3],:], np.array([0, mass]))  # equality constraints C * x = d, yaw moment = zero, thrust = weight

    vertices = np.array(pm.project_polytope(proj, ineq, eq, method='bretl'))
    x.append(list(vertices[:,0]))  # list of x coordinates of vertices for plot
    y.append(list(vertices[:,1]))  # list of y coordinates of vertices for plot

    vertices_fault = np.array(pm.project_polytope(proj, ineq_fault, eq, method='bretl')) 
    x_fault.append(list(vertices_fault[:,0]))
    y_fault.append(list(vertices_fault[:,1]))

    proj = (np.array([[1,0,0,0],[0,1,0,0]]), np.zeros(2))  # 2D projection, roll and pitch
    eq = (np.array([[0,0,1,0],[0,0,0,1]]), np.array([0, mass]))    # equality constraint for yaw = 0 and thrust = weight
    vertices_pseudo = np.array(pm.project_polytope(proj, ineq_f_pseudo, eq, method='bretl'))
    x_pseudo.append(list(vertices_pseudo[:,0]))
    y_pseudo.append(list(vertices_pseudo[:,1]))

    fig, ax = plt.subplots(2, 2)
    colors=['#8DB6CD', '#B80F0A', '#EEE8AA']   

    ax[0,0].grid()
    ax[0,0].fill(x[0], y[0], colors[0], x_fault[0], y_fault[0], colors[1], x_pseudo[0], y_pseudo[0], colors[2])
    ax[0,0].set_xlabel("yaw torque [Nm]")
    ax[0,0].set_ylabel("thrust [kgf]")
    ax[0,0].set_title("L=0, M=0")

    ax[0,1].grid()
    ax[0,1].fill(x[1], y[1], colors[0], x_fault[1], y_fault[1], colors[1], x_pseudo[1], y_pseudo[1], colors[2])
    ax[0,1].set_xlabel("pitch torque [Nm]")
    ax[0,1].set_ylabel("thrust [kgf]")
    ax[0,1].set_title("L=0, N=0")

    ax[1,0].grid()
    ax[1,0].fill(x[2], y[2], colors[0], x_fault[2], y_fault[2], colors[1], x_pseudo[2], y_pseudo[2], colors[2])
    ax[1,0].set_xlabel("roll torque [Nm]")
    ax[1,0].set_ylabel("thrust [kgf]")
    ax[1,0].set_title("M=0, N=0")

    ax[1,1].grid()
    ax[1,1].fill(x[3], y[3], colors[0], x_fault[3], y_fault[3], colors[1], x_pseudo[3], y_pseudo[3], colors[2])
    ax[1,1].set_xlabel("roll torque [Nm]")
    ax[1,1].set_ylabel("pitch torque [Nm]")
    ax[1,1].set_title("N=0, T=mg")

    cc = plt.Circle((0, 0), radius, color='k', linewidth=0.5, fill=False)
    ax[1,1].add_artist(cc)

    fig.tight_layout()
    plt.show()

    return fig

