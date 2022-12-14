from canlib import canlib, Frame

CAN0_ID = 0x000
CAN1_ID = 0x100
KVASER_ID = 0x200

# /*------------------------------------------------------------------*/
# Common:

CHANNEL_NR = 0

# 500kbits, 80% sample
ARBITRATION_PARAMS = canlib.busparams.BusParamsTq(
    tq=80, phase1=63, phase2=16, sjw=4, prescaler=2, prop=0
)
# 4000kbits, 80% sample
DATA_PARAMS = canlib.busparams.BusParamsTq(
    tq=10, phase1=7, phase2=2, sjw=2, prescaler=2, prop=0
)

# Open channels:
rx_channel = canlib.openChannel(channel=CHANNEL_NR, flags=canlib.Open.CAN_FD)
rx_channel.setBusOutputControl(drivertype=canlib.Driver.NORMAL)
rx_channel.set_bus_params_tq(ARBITRATION_PARAMS, DATA_PARAMS)
rx_channel.busOn()

# /*------------------------------------------------------------------*/
# TX/RX:

can0_frame_count = 0  # Frames sent to CAN0
can1_frame_count = 0  # Frames sent to CAN1
unknown_frame_count = 0

rx_i = 0

while True:
    while rx_channel.readStatus() & canlib.Stat.RX_PENDING:
        frame = rx_channel.read()
        if frame.id == KVASER_ID:
            if rx_i % 10_000 == 0:
                print("{} frames successfuly received".format(rx_i))

            i = int.from_bytes(frame.data[:7], "little")
            if rx_i == i and frame.data[7] == 1:
                rx_i += 1
                continue
            else:
                print("invalid rcv: {};{} != {};{}".format(i, frame.data[7], rx_i, 1))
                rx_i = i + 1
        elif frame.id == CAN0_ID:
            can0_frame_count += 1
        elif frame.id == CAN1_ID:
            can1_frame_count += 1
        else:
            unknown_frame_count += 1
            print("ERROR: unknown ID: {}".format(frame.id))
