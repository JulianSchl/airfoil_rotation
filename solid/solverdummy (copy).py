from __future__ import division

import numpy as np
import precice

def angle_between(v1,v2):
    x1 = v1[0]
    x2 = v1[1]
    y1 = v2[0]
    y2 = v2[1]
    dot = x1*x2 + y1*y2      # dot product between [x1, y1] and [x2, y2]
    det = x1*y2 - y1*x2      # determinant
    return np.arctan2(det, dot)  # atan2(y, x) or atan2(sin, cos)

configuration_file_name = "../precice-config.xml"
participant_name = "Solid"
mesh_name = "Solid-Mesh"
write_data_name = 'Displacement'
read_data_name = 'Force'

num_vertices = 2  # Number of vertices

solver_process_index = 0
solver_process_size = 1

interface = precice.Interface(participant_name, configuration_file_name,
                              solver_process_index, solver_process_size)

mesh_id = interface.get_mesh_id(mesh_name)
dimensions = interface.get_dimensions()

vertices = np.zeros((num_vertices, dimensions))
read_data = np.zeros((num_vertices, dimensions))
write_data = np.zeros((num_vertices, dimensions))

vertices = np.array([[0.32, -0.105],[0.32, 0.016 ],[0.32, 0.097]])
read_data = vertices.copy()
write_data = vertices.copy()

vertex_ids = interface.set_mesh_vertices(mesh_id, vertices)
read_data_id = interface.get_data_id(read_data_name, mesh_id)
write_data_id = interface.get_data_id(write_data_name, mesh_id)
print(vertices)
dt = interface.initialize()
iteration = -1
next = 1
while interface.is_coupling_ongoing():
    if interface.is_action_required(
            precice.action_write_iteration_checkpoint()):
        print("DUMMY: Writing iteration checkpoint")
        interface.mark_action_fulfilled(
            precice.action_write_iteration_checkpoint())

    iteration += 1
    if interface.is_read_data_available():
        read_data = interface.read_block_vector_data(read_data_id, vertex_ids)

    print(read_data)
    write_data = np.random.uniform(low=0,high=1*10**-4,size=(3,2))
    #write_data = np.ones((3,2))
    #write_data = read_data + 1
    write_data[:,0] = np.zeros((3))
    if iteration % 1 == 0:
    	next += 1
    
    #translation
    write_data[:,1] = np.ones((3))*0.005*next
    
    
    
    radius = 20
    length = 0.202
    zeit = 10000
    omega = 2*np.pi/zeit
    ohne_disp = np.array((np.cos(omega*iteration)*radius,np.sin(omega*iteration)*radius))
    
    
    
    theta = np.radians(90)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))
    
    norm = R.dot(ohne_disp)
    norm = norm/np.linalg.norm(norm)
    
    write_data[0,:] = np.zeros(2)
    write_data[1,:] = norm*length*0.594
    write_data[2,:] = norm*length
    
    write_data *= 0.05
    
    
    
    '''#rotation
    radius = 20
    length = 0.202
    zeit = 1000000
    omega = 2*np.pi/zeit
    omega = 0
    #ursprung = vertices[0,:] - np.array([radius,0])
    airfoil_ursprung = vertices[0,:]
    circle_ursprung = airfoil_ursprung - np.array([radius,0])
    distance_c_origin_a = np.array([radius,0]) - airfoil_ursprung
    #import ipdb; ipdb.set_trace()
    
    
    
    write_data[0,0] = np.cos(omega*iteration)*radius+circle_ursprung[0]			#x
    write_data[0,1] = np.sin(omega*iteration)*radius+circle_ursprung[1]			#y
    ohne_disp = np.array((np.cos(omega*iteration)*radius,np.sin(omega*iteration)*radius))
    
    
    
    theta = np.radians(90)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))
    
    norm = R.dot(ohne_disp)
    norm = norm/np.linalg.norm(norm)
    
    write_data[1,:] = write_data[0,:]+norm*length*0.594
    write_data[2,:] = write_data[0,:]+norm*length
    #import ipdb; ipdb.set_trace()
    write_data[0,:] = write_data[0,:] - airfoil_ursprung
    write_data[1,:] = write_data[1,:] - airfoil_ursprung 
    write_data[2,:] = write_data[2,:] - airfoil_ursprung '''  
    
    #import ipdb; ipdb.set_trace()
    
    '''	
    #pitch	
    write_data[0,1] = np.ones((1))*-0.005*next*10**-2*10**-2
    write_data[0,0] = np.ones((1))*-0.005*next*10**-2
    write_data[1,1] = np.ones((1))*0
    write_data[1,0] = np.ones((1))*0
    write_data[2,1] = np.ones((1))*0.005*next*10**-2*10**-2
    write_data[2,0] = np.ones((1))*0.005*next*10**-2'''
    
    
    #write_data[:,0] = np.ones((3))*10**-3*(1/iteration**2)
    print(write_data)
    print(np.degrees(omega*iteration))
    

    if interface.is_write_data_required(dt):
        interface.write_block_vector_data(
            write_data_id, vertex_ids, write_data)

    print("DUMMY: Advancing in time")
    dt = interface.advance(dt)

    if interface.is_action_required(
            precice.action_read_iteration_checkpoint()):
        print("DUMMY: Reading iteration checkpoint")
        interface.mark_action_fulfilled(
            precice.action_read_iteration_checkpoint())

interface.finalize()
print("DUMMY: Closing python solver dummy...")
