import asyncio
from bleak import BleakClient


async def relay_control(command):
    address = "c8:f0:9e:47:ed:8e"
    characteristic_uuid = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

    async with BleakClient(address) as client:
        model_number = await client.read_gatt_char(characteristic_uuid)
        print("Model Number: {0}".format("".join(map(chr, model_number))))

        if command == "start":
        # Start the relay loop
            signal_start = "relay_start"
        signal_bytes_start = signal_start.encode('utf-8')
        await client.write_gatt_char(characteristic_uuid, signal_bytes_start)

        if command == "stop":
        # Send the "relay_stop" command
            signal_stop = "relay_stop"
        signal_bytes_stop = signal_stop.encode('utf-8')
        await client.write_gatt_char(characteristic_uuid, signal_bytes_stop)

        if __name__ == "__main__":
            asyncio.run(relay_control("start"))  # Start the relay loop

    if KeyboardInterrupt:
        # When the script is terminated, send the "relay_stop" command
        asyncio.run(relay_control("stop"))