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
from biosiglive import (LivePlot, PlotType, QualisysClient)
from biosiglive import load, RealTimeProcessingMethod, InterfaceType, DeviceType, Server, InverseKinematicsMethods
import asyncio
import logging
import xml.etree.ElementTree as ET
import qtm_rt
import numpy as np
from collections import deque

# Fonction pour détecter si Fz dépasse le seuil
def detect_start(previous_f_z, current_f_z, threshold=30):
    # Détection du passage de inférieur à supérieur au seuil
    return previous_f_z <= threshold < current_f_z





class RealTimeDataProcessor:
    def __init__(self, server_ip="192.168.0.1", port=7,
                 threshold=30, system_rate=100, device_rate=2000, nb_markers=4, nb_seconds=1):
        # Initialisation du serveur
        self.server = Server(server_ip, port)
        self.server.start()
        # Variables d'état
        self.sending_started = False
        self.previous_fz = 0
        self.threshold = threshold

    def load_markers_names(self):
        tmp = load("C:\\Users\\irisse-q\\Desktop\\Florian\\DATA\\LAO_01\\Venue2\\AQM\\LAO_01_Cond0007.qtm")
        return tmp['makers_names'].data[0:self.nb_markers].tolist()

    async def setup(self):
        """ main function """
        # Connection to qualisys
        self.interface = await QualisysClient.create(ip="192.168.0.2", system_rate=100, port=22224)
        queue = asyncio.Queue()

        # Add info needed
        n_markers = 53
        await self.interface.add_marker_set(
            nb_markers=n_markers, data_buffer_size=1000, marker_data_file_key="markers", name="markers", rate=100, unit="mm"
        )
        await self.interface.add_device(
            nb_channels=12,
            device_type="force_plate",
            name="force_plate",
            data_buffer_size=100,
            rate=2000,
            device_data_file_key="force_plate",
            processing_method=None,
            moving_average=True,
        )

        while True:
            tic = asyncio.get_event_loop().time()
            packet = await self.interface.Connect.get_current_frame(components=self.interface.component)

            #data recuperation
            mark_tmp, mks_name = self.interface.get_marker_set_data(packet=packet)


            dataforce = self.interface.get_force_plate_data(packet=packet)

            current_fz = np.mean(dataforce[2])

            if not self.sending_started and detect_start(self.previous_fz, current_fz, self.threshold):
                self.sending_started = True
                print("Démarrage de l'envoi des données.")


            elif self.sending_started:
                connection, message = self.server.client_listening()  # Non-bloquant
                if connection:
                    dataAll = {
                        "Force": dataforce,
                        "Markers": mark_tmp,
                        "MarkersNames": mks_name,
                    }
                    # "Angle": Q[:, -1],
                    self.server.send_data(dataAll, connection, message)

            # Mettre à jour la valeur précédente de Fz
            self.previous_fz = current_fz

            # time laps
            loop_time = asyncio.get_event_loop().time() - tic
            real_time_to_sleep = (1/100) - loop_time
            if real_time_to_sleep > 0:
                await asyncio.sleep(real_time_to_sleep)



if __name__ == "__main__":
    processor = RealTimeDataProcessor()
    # processor.process_data()
    asyncio.run(processor.setup())
    """"
    loop = asyncio.get_event_loop()
    asyncio.ensure_future(setup())
    loop.run_forever()
    """