import asyncio
import websockets
import netifaces as ni
import trim_tab_messages.python.messages_pb2 as message_pb2

ip = ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']
print(ip)

async def echo(websocket, path):
    async for message in websocket:
        try:
            # Deserialize the protobuf message
            protobuf_message = message_pb2.DataMessage()
            protobuf_message.ParseFromString(message)

            print("Received message:"+ str(protobuf_message.windAngle)+": "+str(protobuf_message.batteryLevel))

            # Optionally, send a response back (as a string or protobuf)
            await websocket.send("Message received")
        except Exception as e:
            print("Error processing message:", e)
            raise(e)

start_server = websockets.serve(echo, ip, 8080)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()