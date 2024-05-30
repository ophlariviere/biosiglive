"""
This example shows how to retrieve marker data from a Vicon Nexus interface. Please note that the Vicon interface is the only implemented method capable of retrieving marker data.
First, you need to create a ViconClient object. This object will be used to connect to the Vicon system and retrieve data. Next, you need to add a set of markers to the interface. For now, only one marker set can be added.
The marker set takes the following arguments:
    - nb_markers : int
        Number of markers.
    - name : str
        Name of the marker set.
    - marker_names : Union [list, str]
        List of marker names.
    subject_name : str
        Name of the subject. If None, the subject will be the first in Nexus.
    rate: int
        Rate of the camera used to record the marker trajectories.
    unit : str
        Unit of the marker trajectories ("mm" or "m").
If you want to display the markers in a 3D scatter plot, you can add a Scatter3D plot to the interface. You can pass the size and color of the marker via size and color argument, respectively. Please see the Scatter3D documentation for more information.
Next, the data flow runs in a loop where the get_marker_set_data() function is used to retrieve the data from the interface. The data is then passed to the graph via the update() method through an array of (n_frame, n_markers, 3) where the plot parameters can be updated.
"""
from time import sleep, time
import importlib
import biosiglive
importlib.reload(biosiglive)
from biosiglive import LivePlot, PlotType, QualisysClient
import asyncio
import logging
import xml.etree.ElementTree as ET
import qtm_rt
import numpy as np


async def setup():
    """ main function """
    interface = await QualisysClient.create(ip="192.168.254.1", system_rate=100, port=22224)
    interface._display_info()

    queue = asyncio.Queue()
    n_markers = 4
    await interface.add_marker_set(
        nb_markers=n_markers, data_buffer_size=100, marker_data_file_key="markers", name="markers", rate=100, unit="mm"
    )
    await interface.add_device(
        nb_channels=12,
        device_type="force_plate",
        name="force_plate",
        rate=1000,
        device_data_file_key="force_plate",
        processing_method=None,
        moving_average=True,
    )


    marker_plot = LivePlot(name="markers", plot_type=PlotType.Scatter3D)
    marker_plot.init()
    force1_plot = LivePlot(
        name="force", rate=100, plot_type=PlotType.Curve, nb_subplots=3
    )
    force1_plot.init(plot_windows=500, y_labels="Force (N)")

    force2_plot = LivePlot(
        name="force", rate=100, plot_type=PlotType.Curve, nb_subplots=3
    )
    force2_plot.init(plot_windows=500, y_labels="Force (N)")
    time_to_sleep = 1 / 1000
    offline_count = 0
    mark_to_plot = []

    await interface.Connect.stream_frames(components=interface.component, on_packet=queue.put_nowait)

    while True:
        tic = time()
        packet = await queue.get()
        if packet is None:
            break

        mark_tmp = await interface.get_marker_set_data(packet=packet)
        mark_tmp = mark_tmp / 1000
        marker_plot.update(mark_tmp[:, :, -1].T, size=0.03)
        force_tmp = await interface.get_force_plate_data(packet=packet)
        if (len(force_tmp)) != 0:
            force1_plot.update(np.array(force_tmp[0:3, 0, -1:]))
            force2_plot.update(np.array(force_tmp[0:3, 1, -1:]))
        loop_time = time() - tic
        real_time_to_sleep = time_to_sleep - loop_time
        if real_time_to_sleep > 0:
            sleep(real_time_to_sleep)



if __name__ == "__main__":
    #asyncio.run(setup())

    loop = asyncio.get_event_loop()
    asyncio.ensure_future(setup())
    loop.run_forever()
