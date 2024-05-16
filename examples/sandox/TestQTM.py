""" Example that takes control of QTM, streams data etc """

import asyncio
import logging
import xml.etree.ElementTree as ET
import qtm_rt
import time
LOG = logging.getLogger("example")


async def package_receiver(queue):
    """ Asynchronous function that processes queue until None is posted in queue """
    LOG.info("Entering package_receiver")
    while True:
        packet = await queue.get()
        if packet is None:
            break

        LOG.info("Framenumber %s", packet.framenumber)

        """
        latency = (packet.timestamp) - time.time()
        LOG.info("Latency: %s", latency)
        """

        headermks, mks = packet.get_3d_markers()
        LOG.info("Component info: %s", headermks)
        for i, mks in enumerate(mks, 1):
            LOG.info("Marqueur %d", i)
            for marker in mks:
                LOG.info("\t%s", marker)

        headerf, forces = packet.get_force()
        LOG.info("Component info: %s", headerf)
        for i, forces in enumerate(forces, 1):
            LOG.info("Force %d", i)
            for force in forces:
                LOG.info("\t%s", force)

    LOG.info("Exiting package_receiver")



async def setup():
    """ main function """

    connection = await qtm_rt.connect("192.168.254.1")

    if connection is None:
        return -1

    xlm_general = await connection.get_parameters(parameters=['general'])
    GeneralInfo = ET.fromstring(xlm_general)
    Fs = int(GeneralInfo.find('.//Frequency').text)

    xlm_3d = await connection.get_parameters(parameters=['3d'])
    MksInfo = ET.fromstring(xlm_3d)
    nb_mks = MksInfo.find('.//Labels').text
    nom_mks = [label.find('Name').text for label in MksInfo.findall(".//Label")]
    print(Fs)


    queue = asyncio.Queue()

    receiver_future = asyncio.ensure_future(package_receiver(queue))

    await connection.stream_frames(components=["6d","3d","force"], on_packet=queue.put_nowait)


#        asyncio.ensure_future(shutdown(30, connection, receiver_future, queue))


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    asyncio.ensure_future(setup())
    loop.run_forever()
