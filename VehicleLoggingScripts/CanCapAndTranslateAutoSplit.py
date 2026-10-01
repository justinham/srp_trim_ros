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


    async def readCan():

        # argv[1] = Virtual CAN Network Name
        # argv[2] = DBC Name
        # argv[3] = filePath
        db = cantools.database.load_file('DBC/GlobalA - '+sys.argv[2]+'.dbc')


        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = sys.argv[1]
        can.rc['bitrate'] = 5000000
        

        fNum = 0
        f = open("Data/"+sys.argv[3]+ "/" + sys.argv[2] + "-" + str(fNum) + ".txt", "a")
        logger = can.Logger("Data/"+sys.argv[3]+ "/" + sys.argv[2] + "-" + str(fNum) + ".asc")
        c = 0
        while(CanReader.read):
            with can.Bus() as bus:
                for msg in bus:
                    c += 1
                    logger.log_event(msg)
                    decodedMsg = db.decode_message(msg.arbitration_id, msg.data)
                    msgName = (db.get_message_by_frame_id(msg.arbitration_id)).name
                    f.write(str(hex(msg.arbitration_id)) +"-" +msgName)
                    f.write('\n')
                    for val in decodedMsg.items():
                        f.write("\t"+str(val[0]) + " " + str(val[1]) + "\n")
                    f.write('\n')
                    if c > 50000:
                        c = 0
                        fNum += 1
                        f.close()
                        f = open("Data/"+sys.argv[3]+ "/" + sys.argv[2] + "-" + str(fNum) + ".txt", "a")
                        logger.stop()
                        logger = can.Logger("Data/"+sys.argv[3]+ "/" + sys.argv[2] + "-" + str(fNum) + ".asc")
                        print("FileSplit, Created log number {fNum}\n")


    async def main():

        await asyncio.gather(
            CanReader.waitForInput(),
            CanReader.readCan(),
        )

asyncio.run(CanReader.main())