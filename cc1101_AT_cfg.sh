WIFI_SSID=${WIFI_SSID?Need WIFI_SSID set}
WIFI_PASS=${WIFI_PASS?Need WIFI_PASS set}
d=/dev/ttyACM1
stty -F $d 460800 cs8 -parenb -cstopb -echoe -echok -echoctl -echoke -ixon -ixoff icrnl inlcr ocrnl onlcr -noflsh -opost -isig -icanon -echo
echo -ne "
AT\r
AT+WIFI_PASS=$WIFI_PASS\r
AT+WIFI_SSID=$WIFI_SSID\r
AT+VERBOSE=1\r
AT+DEBUG=0\r
AT+CC1101=1,1,0,868.326447,47.60,0,199.95,812.50,99.97,5,2,211,145,0,0,0,1,0,1,0,0,0,0,0,0,0,0\r
" >> $d

d=/dev/ttyACM2
stty -F $d 460800 cs8 -parenb -cstopb -echoe -echok -echoctl -echoke -ixon -ixoff icrnl inlcr ocrnl onlcr -noflsh -opost -isig -icanon -echo
echo -ne "
AT\r
AT+WIFI_PASS=$WIFI_PASS\r
AT+WIFI_SSID=$WIFI_SSID\r
AT+VERBOSE=1\r
AT+DEBUG=0\r
AT+CC1101=0,1,0,868.326447,47.60,0,199.95,812.50,99.97,5,2,211,145,0,0,0,1,0,1,0,0,0,0,0,0,0,0\r
" >> $d
