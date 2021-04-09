import socket
import json


def main(argv=None):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip_address = s.getsockname()[0]
    s.close()

    print(f'This is the IP address of this host machine: {ip_address}')
    ip_right = input('Look right [Y/n]?')

    if ip_right == 'n':
        ip_address = input('What is the IP address of your host machine? ')

    wifi_ssid = input('What is the name (SSID) of your WiFi network? ')
    wifi_pwd = input(f'What is the WPA password for {wifi_ssid}? ')

    print('')
    print('The following will be used for the configuration file to '
          'set up the ROS 2 interface to the ESP32')
    print('')
    print("Host machine IP address (that's running the Hadabot Docker stack): "
          f"{ip_address}")
    print(f"WiFi SSID: {wifi_ssid}")
    print(f"WiFi WPA password: {wifi_pwd}")
    print("")

    ok = input("The configuration will be written to uhadabot/hb.json "
               "[enter to continue] ")

    hb_config = {
        "ros2_web_bridge_ip_addr": f"{ip_address}",
        "network": {
            "ssid": f"{wifi_ssid}",
            "password": f"{wifi_pwd}"
        }
    }

    hb_json = json.dumps(hb_config, indent=4)
    fp = open('uhadabot/hb.json', 'w')
    fp.write(hb_json)
    fp.close()


if __name__ == "__main__":
    main()
