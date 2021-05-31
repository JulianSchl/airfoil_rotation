from __future__ import division

import numpy as np
from matplotlib import pyplot as plt 
import precice

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

x = 0.32
y_min = -0.1
y_max = 0.09
length = y_max - y_min
x_size = 10001
y_size = 2
zeit = 5*10**3
omega = 2*np.pi/zeit
rot_disp = np.pi/2 #displacement in circle
vertices = np.empty((x_size,y_size))
vertices_relative = np.empty((x_size,y_size)) #from 0 to length
vertices[:,0] = np.ones(x_size)*x
vertices_relative[:,0] = np.zeros(x_size)
vertices[:,1] = np.linspace(y_min,y_max,x_size)
vertices_relative[:,1] = np.linspace(0,length,x_size)
#vertices = np.array([[0.32, -0.1],[0.32, 0.016 ],[0.32, 0.09]])
read_data = vertices.copy()
write_data = vertices.copy()

vertex_ids = interface.set_mesh_vertices(mesh_id, vertices)
read_data_id = interface.get_data_id(read_data_name, mesh_id)
write_data_id = interface.get_data_id(write_data_name, mesh_id)

dt = interface.initialize()
iteration = -1
next = 1
old_time_step = np.zeros((x_size,y_size))
new_time_step = np.zeros((x_size,y_size))
write_data = np.zeros((x_size,y_size))
while interface.is_coupling_ongoing():
    if interface.is_action_required(
            precice.action_write_iteration_checkpoint()):
        print("DUMMY: Writing iteration checkpoint")
        interface.mark_action_fulfilled(
            precice.action_write_iteration_checkpoint())

    iteration += 1
    if interface.is_read_data_available():
        read_data = interface.read_block_vector_data(read_data_id, vertex_ids)

    print("<read data>")
    print(read_data[0:3,:])
    print("</read data>")
    if iteration % 1 == 0:
    	next += 1
    	
    #Translation case
    write_data[:,1] = np.ones((x_size))*0.005*next
    #################
    
    #Rotation case
    #length = 0.202
    
    print("Rotation")
    print(np.degrees(omega*iteration+np.pi/2))
    
    if iteration == 0:
    	for num, line in enumerate(vertices_relative):
    		old_time_step[num,:] = np.array((np.cos(rot_disp)*line[1],np.sin(rot_disp)*line[1]))
    	
    for num, line in enumerate(vertices_relative):
    	new_time_step[num,:] = np.array((np.cos(omega*iteration+rot_disp)*line[1],np.sin(omega*iteration+rot_disp)*line[1]))
    
    displacement = new_time_step.copy() - old_time_step.copy()
    print("Displacement:")
    print(displacement[x_size-3:x_size,:])
    write_data = displacement
    
    #########
    

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
