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

        db = cantools.database.load_file('DBC/GlobalA - '+sys.argv[2]+'.dbc')


        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = sys.argv[1]
        can.rc['bitrate'] = 5000000
        logger = can.Logger("Data/"+sys.argv[2]+sys.argv[3]+ ".asc")

        f = open("Data/"+sys.argv[2]+ "-" + sys.argv[3] + ".txt", "a")

        
        while(CanReader.read):
            with can.Bus() as bus:
                for msg in bus:
                    logger.log_event(msg)
                    decodedMsg = db.decode_message(msg.arbitration_id, msg.data)
                    msgName = (db.get_message_by_frame_id(msg.arbitration_id)).name
                    f.write(str(hex(msg.arbitration_id)) +"-" +msgName)
                    f.write('\n')
                    for val in decodedMsg.items():
                        f.write("\t"+str(val[0]) + " " + str(val[1]) + "\n")
                    f.write('\n')


    async def main():

        await asyncio.gather(
            CanReader.waitForInput(),
            CanReader.readCan(),
        )

asyncio.run(CanReader.main())