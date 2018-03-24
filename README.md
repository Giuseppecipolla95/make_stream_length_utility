# Test of stream length utility
#This code consists of a few tests made up to verify that the stream_length_utility works as expected.
#Test 1: Flow accumulation and direction calculated on a regular grid using the FlowRouter component with D4 method.

from landlab.components import FlowRouter
from landlab import RasterModelGrid
import numpy as np
from landlab.utils.stream_length import calculate_stream_length
from landlab.plot import imshow_grid
from matplotlib.pyplot import figure
from nose.tools import assert_equal

mg = RasterModelGrid((5, 4), spacing=(1, 1))
z = np.array ([[0,0,0,0],[0,21,10,0],[0,31,20,0],[0,32,30,0],[0,0,0,0]],dtype='float64')
#z = np.array ([0.,  0.,  0., 0.,
#             ...0., 21., 10., 0.,
#             ...0., 31., 20., 0.,
#             ...0., 32., 30., 0.,
#            ...0.,  0.,  0., 0.])
_ = mg.add_field('node','topographic__elevation', z)
figure(); imshow_grid(mg,mg['node']['topographic__elevation'])

flow_length_expected = np.array ([[0,0,0,0],[0,1,0,0],[0,2,1,0],[0,3,2,0],[0,0,0,0]],dtype='float64')


mg.set_closed_boundaries_at_grid_edges(bottom_is_closed=True, left_is_closed=True, right_is_closed=True, top_is_closed=True)

fr = FlowRouter(mg, method = 'D4')
fr.route_flow()

stream__length = calculate_stream_length(mg, add_to_grid=True, noclobber=False)

#Boundary nodes and the outlet shouldn't have stream_length value different from 0
flow_length = np.reshape(stream__length-stream__length[6],mg.shape)
mg.add_field('node','flow_length',flow_length)
figure(); imshow_grid(mg,mg['node']['flow_length'])

assert_equal (flow_length_expected.all(),flow_length.all(),mg)

#Test 2: Flow accumulation and direction calculated on a regular grid using the FlowRouter component with D8 method.
from landlab.components import FlowRouter
from landlab import RasterModelGrid
import numpy as np
from landlab.utils.stream_length import calculate_stream_length
from landlab.plot import imshow_grid
from matplotlib.pyplot import figure
from nose.tools import assert_equal
import math


mg = RasterModelGrid((5, 4), spacing=(1, 1))
z = np.array ([[0,0,0,0],[0,21,10,0],[0,31,20,0],[0,32,30,0],[0,0,0,0]],dtype='float64')
_ = mg.add_field('node','topographic__elevation', z)
figure(); imshow_grid(mg,mg['node']['topographic__elevation'])

flow_length_expected = np.array ([[0,0,0,0],[0,1,0,0],[0,math.sqrt(2),1,0],[0,1+math.sqrt(2),2,0],[0,0,0,0]],dtype='float64')


mg.set_closed_boundaries_at_grid_edges(bottom_is_closed=True, left_is_closed=True, right_is_closed=True, top_is_closed=True)

fr = FlowRouter(mg, method = 'D8')
fr.route_flow()

stream__length = calculate_stream_length(mg, add_to_grid=True, noclobber=False)

#Boundary nodes and the outlet shouldn't have stream_length value different from 0
flow_length = np.reshape(stream__length-stream__length[6],mg.shape)
mg.add_field('node','flow_length',flow_length)
figure(); imshow_grid(mg,mg['node']['flow_length'])

assert_equal (flow_length_expected.all(),flow_length.all(),mg)


#Test 3: Flow accumulation and direction calculated on a irregular grid using the FlowRouter component with D4 method.
from landlab import HexModelGrid, CLOSED_BOUNDARY
from landlab.components import FlowRouter
import numpy as np
from landlab.utils.stream_length import calculate_stream_length
from landlab.plot import imshow_grid
from matplotlib.pyplot import figure

dx=(2./(3.**0.5))**0.5
hmg = HexModelGrid(5,3, dx)
_ = hmg.add_field('topographic__elevation', hmg.node_x + np.round(hmg.node_y), at = 'node')
figure(); imshow_grid(hmg,hmg['node']['topographic__elevation'])
hmg.status_at_node[0] = CLOSED_BOUNDARY

fr = FlowRouter(hmg, method = 'D4') 
fr.route_flow()

stream__length = calculate_stream_length(hmg, add_to_grid=True, noclobber=False)

#Outlet shouldn't have stream_length value different from 0
flow_length = stream__length-stream__length[4]
hmg.add_field('node','flow_length',flow_length)
figure(); imshow_grid(hmg,hmg['node']['flow_length'])


#Test 4: Flow accumulation and direction calculated on a irregular grid using the FlowRouter component with D8 method.
from landlab import HexModelGrid, CLOSED_BOUNDARY
from landlab.components import FlowRouter
import numpy as np
from landlab.utils.stream_length import calculate_stream_length
from landlab.plot import imshow_grid
from matplotlib.pyplot import figure

dx=(2./(3.**0.5))**0.5
hmg = HexModelGrid(5,3, dx)
_ = hmg.add_field('topographic__elevation', hmg.node_x + np.round(hmg.node_y), at = 'node')
figure(); imshow_grid(hmg,hmg['node']['topographic__elevation'])
hmg.status_at_node[0] = CLOSED_BOUNDARY

fr = FlowRouter(hmg, method = 'D8') 
fr.route_flow()

stream__length = calculate_stream_length(hmg, add_to_grid=True, noclobber=False)

#Outlet shouldn't have stream_length value different from 0
flow_length = stream__length-stream__length[4]
hmg.add_field('node','flow_length',flow_length)
figure(); imshow_grid(hmg,hmg['node']['flow_length'])
