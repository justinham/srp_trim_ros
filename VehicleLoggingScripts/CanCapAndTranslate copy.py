import asyncio
import can
import cantools
import sys



class CanReader:
    read = True
    async def waitForInput():
        user_input = ""
        user_input = await asyncio.to_thread(input, "Press enter to quit program\n")
        
        CanReader.read = False
        return False


    async def readCan(canNet, netName, logName):
        print(canNet)
        dbFO = cantools.database.load_file('DBC/GlobalA - '+netName+'.dbc')


        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = canNet
        can.rc['bitrate'] = 5000000
        logger = can.Logger("Data/"+netName+ "-" + logName+ ".asc")

        f = open("Data/"+netName+ "-" + logName + ".txt", "a")


        while(CanReader.read):
            with can.Bus() as bus:
                for msg in bus:
                    logger.log_event(msg)
                    decodedMsg = dbFO.decode_message(msg.arbitration_id, msg.data)
                    f.write(str(hex(msg.arbitration_id)))
                    f.write('\n')
                    for val in decodedMsg.items():
                        f.write("\t"+str(val[0]) + " " + str(val[1]) + "\n")
                    f.write('\n')

    async def readCan2(canNet, netName, logName):
        print(canNet)
        dbFO = cantools.database.load_file('DBC/GlobalA - '+netName+'.dbc')


        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = canNet
        can.rc['bitrate'] = 5000000
        logger = can.Logger("Data/"+netName+ "-" + logName+ ".asc")

        f = open("Data/"+netName+ "-" + logName + ".txt", "a")


        while(CanReader.read):
            with can.Bus() as bus:
                for msg in bus:
                    logger.log_event(msg)
                    decodedMsg = dbFO.decode_message(msg.arbitration_id, msg.data)
                    f.write(str(hex(msg.arbitration_id)))
                    f.write('\n')
                    for val in decodedMsg.items():
                        f.write("\t"+str(val[0]) + " " + str(val[1]) + "\n")
                    f.write('\n')


    async def main():

        # Schedule three calls *concurrently*:
        L = await asyncio.gather(
            CanReader.waitForInput(),
            #CanReader.readCan('can0', 'FO', sys.argv[1]),
            CanReader.readCan2('can1', 'CE', sys.argv[1])
            #CanReader.readCan('can2', 'HS', sys.argv[1]),
        )
        print(L)

asyncio.run(CanReader.main())