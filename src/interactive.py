#! /usr/env/bin python3

"""Launch script for running robot code simulation with support for input
from Arbotix controller and live plotting of both controller input and
positions generated for servos 
"""

from argparse import ArgumentParser
from serial import Serial, SerialException
from time import sleep
from queue import Queue
from threading import Thread as Process
#from multiprocessing import Queue, Process

from mock_hardware import MockUART_Port_from_COM, MockBusToQueue

from pydypackets.PyDyLogger import LoggerClass
from pydypackets.PyDyPlotter import Plotter

from numa import NumaMain
from poses import gen_numa2_legs#, g8Stand, g8FeetDown, g8Flop, g8Crouch
from IK import Gaits


def main(args):

    while True:
        # Opening the serial port to read Commander input
        try:
            # TODO maybe we want to set a timeout arg (default None)
            ser = Serial(args.port, args.baud)
            print("Successfully connected to port {0} at {1} baud".format(args.port, args.baud))
            break

        except SerialException as e:
            print("Couldn't open serial port {0}. Trying again in 5 s...".format(args.port))
            #print(e)
            sleep(5)


##    import matplotlib
##    print(matplotlib.get_backend())
#    from matplotlib import pyplot as plt
#    import numpy as np
##    # plt.ion() # doesn't fix the matplotlib icon error
##    plt.figure()
##    plt.show()
##    print("...")
#    plt.axis([-50,50,-10,10])
#    plt.ion()
#    plt.show()
#
#    x = [0,1]
#    y = np.array([-1,-1])
#    myline = plt.plot(x, y)[-1]
#    print("Myline:", myline)
#    #x = np.arange(-50, 51)
#    while True:
#        for pow in range(1,5):   # plot x^1, x^2, ..., x^4
#            #y = [Xi**pow for Xi in x]
#            y = np.append(y, pow)
#            x = np.array(range(len(y)))
#            myline.set_xdata(x)
#            myline.set_ydata(y)
#            #print(x)
#            #print(y)
#            
#            plt.draw()
#            plt.pause(0.001)
#            sleep(.1)
#            #input("Press [enter] to continue.")
#    return

    uart_from_cmdr = MockUART_Port_from_COM(ser)
    axbus = MockBusToQueue()
    axqueue = axbus.get_queue()

    leg_geom, leg1, leg2, leg3, leg4 = gen_numa2_legs()
    gaits = Gaits(leg_geom, leg1, leg2, leg3, leg4)

    numa = NumaMain(gaits, cmdrbus=uart_from_cmdr, axbus=axbus)

    # Hardcoding these, which are normally config params for PyDyPackets
    # TODO confirm these
    args.saveall = True
    args.translate = False#True
    args.timing = True
    #
    lgr = LoggerClass(args)
    plotqueue = Queue()
    process_robot = Process(target=numa.main, args=(leg_geom,), daemon=True)
    ###process_lgr = Process(target=lgr.logger_method, kwargs={"read_queue": axqueue}, daemon=True) # writes to file
    process_lgr = Process(target=lgr.logger_method, kwargs={"read_queue": axqueue, "write_queue": plotqueue}, daemon=True)
    #process_plotter = Process(target=lgr.logger_method, kwargs={"read_queue": axqueue, "write_queue": plotqueue}, daemon=True)

    print(axqueue, axbus.get_queue())
    print("AXQUEUE:", axqueue)
    input("Ready to start simulation... (press enter)")
    process_robot.start()
    process_lgr.start()
    # Wait a second for us to get some packets so that we know all servos receiving positions
    # from the robot process. This ensures the plotting_dict is properly populated when we create it.
    sleep(2)
    print("Started!")

    # Get an initial set of packets to create our Plotter with
    new_packets = []
    # TODO arbitrary; hope we get fewer than 3000 packets in that 1 second...
    while not plotqueue.empty() and len(new_packets) < 3000:
        new_packets.append(plotqueue.get())
    print("Initial packets from plotqueue:", len(new_packets))

    if not args.no_plot:
        pltr = Plotter() # TODO
        #print("happen yet?")
        id_list = [11,12,13,21,22,23,31,32,33,41,42,43,51,52] # Full set
        id_list = [11,12,13] # Partial instead
        pltr.plot_trends(new_packets, id_list=id_list, make_plot=True, axis_extents=[0,200,100,900])#False)
        #print("happen yet? yeah")

    try:
        #input("...")
        while True:
            sleep(.1)
            print("Empty?", axqueue.empty(), plotqueue.empty())
            print("packet cnts:",
                    "lgr bytes read", lgr.readbytecnt,
                    "\tlgr read packets", lgr.readpacketcnt,
                    "\tlgr saved packets", lgr.loggedpacketcnt)
            #print(process_robot.is_alive(), process_lgr.is_alive(), axqueue.qsize())

            new_packets = []
            # NOTE: Arbitrary packet batch size limit...
            while not plotqueue.empty() and len(new_packets) < 30:
                new_packets.append(plotqueue.get())

            if not args.no_plot:
                print("new packets from plotqueue:", len(new_packets))
                pltr.parse_packets(new_packets)

                pltr.callback()
    except KeyboardInterrupt:
        print("Shutting down gracefully (TODO)")



if __name__ == '__main__':
    usage = "usage: %(prog)s [options]"
    parser = ArgumentParser(prog='pydylogger', usage=usage,
                            description="TODO",
                            )#formatter_class=RawTextHelpFormatter)

    parser.add_argument('-p', '--port', action="store",  default="COM6",
            help="COM/serial port to read commander packets from. Default: %(default)s")
    parser.add_argument('-b', '--baud', action="store",
            default=38400, help="Baud rate to use for serial port. Default: %(default)s")
    parser.add_argument('-n', '--no-plot', action="store_true",
            help="Don't show/create plot. Use for faster/smoother simulation")

    args = parser.parse_args()

    main(args)
