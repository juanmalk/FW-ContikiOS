<?xml version="1.0" encoding="UTF-8"?>
<simconf>
  <project EXPORT="discard">[APPS_DIR]/mrm</project>
  <project EXPORT="discard">[APPS_DIR]/mspsim</project>
  <project EXPORT="discard">[APPS_DIR]/avrora</project>
  <project EXPORT="discard">[APPS_DIR]/serial_socket</project>
  <project EXPORT="discard">[APPS_DIR]/collect-view</project>
  <project EXPORT="discard">[APPS_DIR]/powertracker</project>
  <simulation>
    <title>My simulation</title>
    <randomseed>123456</randomseed>
    <motedelay_us>1000000</motedelay_us>
    <radiomedium>
      org.contikios.cooja.radiomediums.UDGM
      <transmitting_range>50.0</transmitting_range>
      <interference_range>50.0</interference_range>
      <success_ratio_tx>1.0</success_ratio_tx>
      <success_ratio_rx>1.0</success_ratio_rx>
    </radiomedium>
    <events>
      <logoutput>40000</logoutput>
    </events>
    <motetype>
      org.contikios.cooja.contikimote.ContikiMoteType
      <identifier>mtype710</identifier>
      <description>Sender</description>
      <source>[CONTIKI_DIR]/regression-tests/12-rpl/code/sender-node.c</source>
      <commands>make TARGET=cooja clean
make sender-node.cooja TARGET=cooja</commands>
      <moteinterface>org.contikios.cooja.interfaces.Position</moteinterface>
      <moteinterface>org.contikios.cooja.interfaces.Battery</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiVib</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiMoteID</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiRS232</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiBeeper</moteinterface>
      <moteinterface>org.contikios.cooja.interfaces.RimeAddress</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiIPAddress</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiRadio</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiButton</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiPIR</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiClock</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiLED</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiCFS</moteinterface>
      <moteinterface>org.contikios.cooja.interfaces.Mote2MoteRelations</moteinterface>
      <moteinterface>org.contikios.cooja.interfaces.MoteAttributes</moteinterface>
      <symbols>false</symbols>
    </motetype>
    <motetype>
      org.contikios.cooja.contikimote.ContikiMoteType
      <identifier>mtype709</identifier>
      <description>RPL root</description>
      <source>[CONTIKI_DIR]/regression-tests/12-rpl/code/root-node.c</source>
      <commands>make TARGET=cooja clean
make root-node.cooja TARGET=cooja</commands>
      <moteinterface>org.contikios.cooja.interfaces.Position</moteinterface>
      <moteinterface>org.contikios.cooja.interfaces.Battery</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiVib</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiMoteID</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiRS232</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiBeeper</moteinterface>
      <moteinterface>org.contikios.cooja.interfaces.RimeAddress</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiIPAddress</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiRadio</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiButton</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiPIR</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiClock</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiLED</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiCFS</moteinterface>
      <moteinterface>org.contikios.cooja.interfaces.Mote2MoteRelations</moteinterface>
      <moteinterface>org.contikios.cooja.interfaces.MoteAttributes</moteinterface>
      <symbols>false</symbols>
    </motetype>
    <motetype>
      org.contikios.cooja.contikimote.ContikiMoteType
      <identifier>mtype332</identifier>
      <description>Receiver</description>
      <source>[CONTIKI_DIR]/regression-tests/12-rpl/code/receiver-node.c</source>
      <commands>make TARGET=cooja clean
make receiver-node.cooja TARGET=cooja</commands>
      <moteinterface>org.contikios.cooja.interfaces.Position</moteinterface>
      <moteinterface>org.contikios.cooja.interfaces.Battery</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiVib</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiMoteID</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiRS232</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiBeeper</moteinterface>
      <moteinterface>org.contikios.cooja.interfaces.RimeAddress</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiIPAddress</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiRadio</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiButton</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiPIR</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiClock</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiLED</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiCFS</moteinterface>
      <moteinterface>org.contikios.cooja.interfaces.Mote2MoteRelations</moteinterface>
      <moteinterface>org.contikios.cooja.interfaces.MoteAttributes</moteinterface>
      <symbols>false</symbols>
    </motetype>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>-15.604889524290883</x>
        <y>-27.272920930192623</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>1</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>218.29521040499824</x>
        <y>216.70287561143516</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>2</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype710</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>0.0</x>
        <y>0.0</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>3</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype709</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>27.44274795318258</x>
        <y>36.980443988209856</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>4</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>185.6055859234863</x>
        <y>192.99166984979271</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>5</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>7.827315175624361</x>
        <y>107.95833747378225</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>6</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>82.49199862549197</x>
        <y>34.72851022020231</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>7</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>35.671428723363064</x>
        <y>125.82590119075962</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>8</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>33.044376889885754</x>
        <y>62.443378727185774</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>9</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>152.53659733643553</x>
        <y>165.87608708149403</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>10</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>22.023077232445942</x>
        <y>18.41094139254531</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>11</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>41.19368867821842</x>
        <y>49.201157808285224</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>12</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>114.62128502432532</x>
        <y>88.58807114217633</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>13</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>61.21405469314478</x>
        <y>188.6851979777661</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>14</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>104.78326932115709</x>
        <y>100.84889281105585</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>15</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>90.26861048950052</x>
        <y>40.550864496421404</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>16</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>137.39112328314076</x>
        <y>126.5333637365394</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>17</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>14.07616766247768</x>
        <y>99.85572366133869</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>18</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>192.30563307960443</x>
        <y>66.83563821262344</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>19</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>129.08253247651874</x>
        <y>117.20437669309078</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>20</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>21.82717410659969</x>
        <y>47.523771402541335</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>21</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>35.95102103988239</x>
        <y>21.74677482276881</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>22</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>61.71108896268191</x>
        <y>79.91617509627334</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>23</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>16.029809485043288</x>
        <y>171.05935835280616</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>24</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>138.8418552766128</x>
        <y>61.418703448852185</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>25</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>151.23299942414673</x>
        <y>92.41820871538522</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>26</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>197.33157775573343</x>
        <y>20.6013482653864</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>27</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>121.20762229879878</x>
        <y>184.81107462083565</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>28</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>102.32432527534694</x>
        <y>91.9912435435037</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>29</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>88.10909646999544</x>
        <y>191.21251904898142</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>30</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>72.19774934085703</x>
        <y>113.58131529956069</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>31</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>53.49967737007204</x>
        <y>72.45172156882643</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>32</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>172.07186411958625</x>
        <y>51.47715718716961</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>33</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>185.41983532466634</x>
        <y>85.60078269414</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>34</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>83.34993971740548</x>
        <y>193.98111239532744</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>35</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>105.03752362550378</x>
        <y>131.24078026424087</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>36</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>176.09318670322102</x>
        <y>41.46760901468247</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>37</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>76.80689915307215</x>
        <y>47.13815728542057</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>38</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>155.907739287817</x>
        <y>15.24009422994106</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>39</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>45.992463430523436</x>
        <y>124.573811024683</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>40</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>159.11361032671832</x>
        <y>81.65319598335425</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>41</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>41.838657583001314</x>
        <y>163.47591213471193</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>42</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>25.560001904073125</x>
        <y>147.48610852243928</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>43</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>139.56489350213488</x>
        <y>191.15379557180913</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>44</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>105.71342550700061</x>
        <y>136.09089690661503</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>45</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>122.59378462201298</x>
        <y>196.22862961645998</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>46</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>147.61372446125088</x>
        <y>55.314287700435536</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>47</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>77.70218628780312</x>
        <y>95.59561907133107</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>48</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>71.88244578562713</x>
        <y>168.57963926907163</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>49</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>6.722298767036894</x>
        <y>101.09668965729898</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>50</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>62.96680217979964</x>
        <y>77.66951408660954</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>51</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>162.49011735601982</x>
        <y>199.07086470932003</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>52</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>87.87526390617558</x>
        <y>114.39424478293958</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>53</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>74.29230372180815</x>
        <y>36.995699473573836</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>54</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>77.34619341407321</x>
        <y>60.070446577058576</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>55</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>76.32695571818826</x>
        <y>135.82669004433725</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>56</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>68.10326013650814</x>
        <y>5.04157445893032</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>57</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>95.76993029214962</x>
        <y>45.046282401332945</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>58</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>79.87963205080952</x>
        <y>110.7023948653882</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>59</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>195.90349780899223</x>
        <y>132.38904172009444</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>60</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>80.40108440304007</x>
        <y>25.86673418569547</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>61</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>52.268877618080744</x>
        <y>176.12888723955277</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>62</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>40.545541404899765</x>
        <y>166.9450252729589</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>63</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>10.676184465528205</x>
        <y>0.9871174057552334</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>64</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>87.35673216830257</x>
        <y>23.25131234780027</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>65</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>26.745126931691352</x>
        <y>87.3101256645554</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>66</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>183.2724008541638</x>
        <y>167.69026002459069</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>67</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>5.934720840855223</x>
        <y>77.21248812623693</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>68</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>76.49604470599058</x>
        <y>108.80963795015302</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>69</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>86.10145414955488</x>
        <y>12.798582653303603</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>70</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>32.435370110193816</x>
        <y>29.344591306898813</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>71</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>163.72950518161596</x>
        <y>154.15283820759655</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>72</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>143.96396308843373</x>
        <y>132.40312602892786</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>73</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>139.20530179839795</x>
        <y>144.18958011498225</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>74</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>75.22453368236212</x>
        <y>119.69913560274786</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>75</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>21.83836116635087</x>
        <y>191.21696522067728</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>76</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>187.94640511976667</x>
        <y>113.95684826994561</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>77</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>114.61289565250618</x>
        <y>27.61001303446735</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>78</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>70.83569869750504</x>
        <y>113.88992677636021</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>79</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>11.616059362048992</x>
        <y>45.59401384110464</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>80</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>3.4282454263252937</x>
        <y>35.97653370545284</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>81</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>54.33122057405715</x>
        <y>1.9759087814547494</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>82</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>69.6612091444155</x>
        <y>0.45982758130533874</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>83</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>169.98417155202168</x>
        <y>87.76678058442593</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>84</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>148.98441194234616</x>
        <y>104.32820155415494</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>85</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>66.23883124891583</x>
        <y>162.92685536074129</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>86</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>181.6348837921769</x>
        <y>183.07267240808343</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>87</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>88.9829570206748</x>
        <y>119.72520333459575</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>88</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>179.1527012143494</x>
        <y>84.25685075020328</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>89</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>117.82537007493707</x>
        <y>41.10319533518651</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>90</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>52.611633540398486</x>
        <y>94.71918054798351</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>91</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>128.22088664324633</x>
        <y>115.50290142480077</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>92</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>102.53841128002531</x>
        <y>11.784449645612295</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>93</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>187.69871925603667</x>
        <y>131.28317666772048</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>94</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>71.13897508938263</x>
        <y>106.29335632876602</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>95</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>71.2216469861295</x>
        <y>148.38612859488788</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>96</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>36.152562577928784</x>
        <y>67.541796718348</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>97</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>118.84793016344604</x>
        <y>0.49906433835273933</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>98</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>135.20417096106954</x>
        <y>170.20704631856816</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>99</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>156.082359043526</x>
        <y>57.62103450217495</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>100</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>180.58695188377737</x>
        <y>80.75266645775669</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>101</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>38.93889890269273</x>
        <y>138.60259660238856</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>102</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>159.5172788118917</x>
        <y>192.24950143209503</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>103</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>154.02096825135868</x>
        <y>139.67175722659056</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>104</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>38.480826888323904</x>
        <y>5.502866276505292</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>105</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>27.95908088015704</x>
        <y>193.85188308665965</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>106</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>198.61700949829074</x>
        <y>171.1877312716402</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>107</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>106.30468818084609</x>
        <y>25.058380770654654</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>108</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>40.335672117825624</x>
        <y>179.59080234641004</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>109</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>22.316357638510897</x>
        <y>158.94323888090028</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>110</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>99.56281553229194</x>
        <y>85.64260133077535</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>111</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>60.71556414510035</x>
        <y>121.53040248649711</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>112</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>27.08600745576586</x>
        <y>38.17025720346818</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>113</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>147.03642509910586</x>
        <y>39.51320588416096</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>114</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>180.2031547656297</x>
        <y>141.5646561330238</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>115</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>73.12314629214424</x>
        <y>167.80783320779847</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>116</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>167.93567452922767</x>
        <y>10.060141001139144</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>117</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>163.99198875105768</x>
        <y>147.48735207074026</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>118</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>154.98654737808127</x>
        <y>121.17266324007643</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>119</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>57.898499791676386</x>
        <y>149.3487194893122</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>120</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>64.82082963563904</x>
        <y>174.0100480348673</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>121</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>114.13623920898752</x>
        <y>15.754246175503095</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>122</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>0.00848902940355778</x>
        <y>195.50701335573908</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>123</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>182.81764401709623</x>
        <y>78.57837811285111</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>124</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>11.462145876504714</x>
        <y>95.37444120802225</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>125</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>115.5020283241771</x>
        <y>28.49431396939579</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>126</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>62.463952359877275</x>
        <y>77.78913013330184</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>127</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>9.766778039117696</x>
        <y>136.7421944039438</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>128</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>35.320514349220055</x>
        <y>100.56248258192493</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>129</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>56.26732169234614</x>
        <y>3.095097140731262</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>130</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>76.90164393617998</x>
        <y>3.5671096386384216</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>131</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>81.97549075841862</x>
        <y>13.020422155003475</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>132</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>54.87200208203389</x>
        <y>77.29445717372947</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>133</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>193.47651032749548</x>
        <y>144.9357554583657</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>134</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>51.65288193591992</x>
        <y>126.16687604535504</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>135</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>151.66849746442173</x>
        <y>158.7699863850836</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>136</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>32.469410974826005</x>
        <y>113.10993021698361</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>137</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>33.04622512107349</x>
        <y>25.425445944702794</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>138</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>33.53444748873715</x>
        <y>112.25721598241579</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>139</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>128.9401580272291</x>
        <y>100.73482184242926</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>140</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>194.6592727528704</x>
        <y>102.73664509470841</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>141</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>125.47343036050516</x>
        <y>106.53155237731285</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>142</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>147.40129296416038</x>
        <y>12.37607345376115</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>143</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>113.32045045397959</x>
        <y>126.79285457987103</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>144</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>27.174837677715825</x>
        <y>66.84349985536826</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>145</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>164.20252670136998</x>
        <y>51.635539499142524</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>146</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>39.351673884988394</x>
        <y>65.05462325698123</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>147</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>143.91486202542433</x>
        <y>171.28435110465497</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>148</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>30.926988343440186</x>
        <y>130.3647571119649</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>149</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>19.897357003413617</x>
        <y>22.905473451246785</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>150</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>128.36369031733133</x>
        <y>170.71462512320227</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>151</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>172.1648617546042</x>
        <y>184.1928317689217</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>152</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>148.16672573170842</x>
        <y>107.99684523382686</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>153</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>83.90300186263724</x>
        <y>169.4761782218257</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>154</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>90.86887922361453</x>
        <y>142.8036645841288</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>155</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>23.18966921326753</x>
        <y>69.42635534680753</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>156</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>134.59433377860168</x>
        <y>37.633119904417796</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>157</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>127.97132285920065</x>
        <y>158.57917470101572</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>158</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>166.2450456558778</x>
        <y>108.67197275397042</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>159</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>167.334034200758</x>
        <y>22.173554305333166</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>160</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>163.54194527237107</x>
        <y>189.41605447848966</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>161</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>14.863357513573018</x>
        <y>93.36644051662617</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>162</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>178.4370382798651</x>
        <y>191.48348446587636</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>163</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>172.96087703915273</x>
        <y>183.78535300027013</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>164</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>2.45779137738229</x>
        <y>58.750309492130114</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>165</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>15.730601618735495</x>
        <y>96.52335072173565</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>166</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>19.890833428932165</x>
        <y>56.993000152370364</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>167</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>154.69166608840504</x>
        <y>164.8598339150269</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>168</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>199.77196118880582</x>
        <y>26.034321005016903</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>169</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>5.216343266336931</x>
        <y>17.867912968799615</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>170</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>138.71273406283296</x>
        <y>55.31024592694844</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>171</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>168.21144361519595</x>
        <y>163.1843284579423</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>172</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>61.82504434646854</x>
        <y>134.03197926926038</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>173</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>14.007317972338473</x>
        <y>146.98475859141027</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>174</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>108.44210606040488</x>
        <y>127.81076428732186</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>175</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>133.41957560864708</x>
        <y>122.91534078890439</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>176</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>184.89266828168118</x>
        <y>195.7201293014503</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>177</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>121.10492556426465</x>
        <y>78.54418709376823</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>178</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>42.05271519544296</x>
        <y>183.14259881514795</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>179</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>92.12119616387746</x>
        <y>44.853464007589714</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>180</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>136.44039199596196</x>
        <y>1.111619893261917</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>181</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>64.37440878278696</x>
        <y>188.3078368775181</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>182</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>37.774149552594594</x>
        <y>73.81683900641865</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>183</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>24.618589360988175</x>
        <y>164.89708336795567</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>184</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>152.5950797265111</x>
        <y>140.96774353352123</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>185</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>37.96933854365052</x>
        <y>131.92434845994435</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>186</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>14.05145808951862</x>
        <y>26.159084136809916</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>187</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>61.47558158857349</x>
        <y>68.73507104275693</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>188</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>35.420404733112385</x>
        <y>108.47794695541302</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>189</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>129.0179255565185</x>
        <y>176.46977408461998</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>190</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>4.437687657989087</x>
        <y>191.43801818673953</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>191</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>132.39232886927158</x>
        <y>105.56546037448346</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>192</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>193.9356389936735</x>
        <y>76.8987220921185</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>193</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>123.49672189705024</x>
        <y>16.28922647444049</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>194</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>40.44058024960566</x>
        <y>94.77629608096818</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>195</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>93.54168452285269</x>
        <y>102.10342793159373</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>196</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>176.91858637781746</x>
        <y>81.80773827257306</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>197</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>182.5062159995403</x>
        <y>10.047631291589564</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>198</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>151.3211952231698</x>
        <y>160.98807517681706</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>199</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>167.8826595707132</x>
        <y>160.3686256248768</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>200</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>93.38491122055773</x>
        <y>61.04322963139093</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>201</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>155.20211039479165</x>
        <y>104.99915002371228</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>202</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>154.56882959476744</x>
        <y>192.77647809954323</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>203</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>119.88029558288524</x>
        <y>48.71837870038327</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>204</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>8.237800167806908</x>
        <y>123.56280331420268</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>205</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>53.424877837249696</x>
        <y>87.33638375233281</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>206</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>199.25320093864096</x>
        <y>66.76343110330383</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>207</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>166.7255674935148</x>
        <y>165.31992478582075</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>208</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>135.9334286576811</x>
        <y>130.36986226660702</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>209</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>65.60950768388696</x>
        <y>14.081940005079275</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>210</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>56.68133966983844</x>
        <y>196.61338214776293</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>211</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>2.562777529582072</x>
        <y>66.73129709411079</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>212</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>34.10919172462936</x>
        <y>176.31986637140767</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>213</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>33.846173186403306</x>
        <y>142.33660392777279</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>214</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>128.76326079790124</x>
        <y>90.05136184351706</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>215</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>80.39617891964872</x>
        <y>111.48714907972395</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>216</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>21.97338508940303</x>
        <y>61.032785792453815</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>217</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>158.00182839254427</x>
        <y>175.79991821384095</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>218</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>64.21702017661488</x>
        <y>197.8650659620092</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>219</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>58.261430108425174</x>
        <y>69.56229252260285</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>220</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>114.85635789777962</x>
        <y>130.3021189977344</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>221</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>24.601661057872736</x>
        <y>196.33046445845073</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>222</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>136.22037670133446</x>
        <y>18.019579846123435</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>223</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>71.91168973841357</x>
        <y>193.3123397692768</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>224</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>76.67497322258676</x>
        <y>156.30488619961912</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>225</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>85.70841880772959</x>
        <y>39.914700874835106</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>226</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>142.3427145819395</x>
        <y>76.80244108802334</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>227</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>45.6821653485997</x>
        <y>33.96733547026549</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>228</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>146.16353499121948</x>
        <y>5.58117117441268</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>229</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>34.32680631388174</x>
        <y>111.76495490887346</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>230</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>138.5676204843924</x>
        <y>161.4559204389253</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>231</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>196.36096598095253</x>
        <y>19.9809316402896</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>232</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>190.5761031572042</x>
        <y>118.16570999859783</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>233</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>137.89894164409372</x>
        <y>114.36842366282201</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>234</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>13.942927177934262</x>
        <y>25.042265908173977</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>235</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>21.808225381827363</x>
        <y>89.51408500063611</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>236</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>82.1665299576559</x>
        <y>89.41818802221223</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>237</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>100.24631818801446</x>
        <y>85.16089691564225</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>238</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>150.87166478995505</x>
        <y>124.32687790892683</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>239</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>196.72953604773366</x>
        <y>89.0589559016778</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>240</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>178.10423185724105</x>
        <y>108.01295472332721</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>241</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>24.852336661830865</x>
        <y>107.10027825925053</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>242</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>198.2838988728342</x>
        <y>185.2533889301396</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>243</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>122.42366366542863</x>
        <y>13.685145107609165</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>244</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>113.58516359448151</x>
        <y>59.212889358544054</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>245</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>148.6689453992514</x>
        <y>65.76664113968091</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>246</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>110.90604811956779</x>
        <y>118.12368970120251</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>247</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>195.48877933255176</x>
        <y>71.61703558744803</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>248</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>90.74911177135543</x>
        <y>151.9662290090636</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>249</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>5.669662023293154</x>
        <y>80.71705944385323</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>250</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>77.57071934873466</x>
        <y>25.884947016521597</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>251</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>172.38535892291588</x>
        <y>6.827522113737539</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>252</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>62.61595677732154</x>
        <y>171.52926299696654</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>253</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>163.1605182212256</x>
        <y>136.67511261098457</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>254</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>174.98684717123461</x>
        <y>163.9648526025463</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>255</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>112.4179766207543</x>
        <y>108.05999669756379</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>256</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>71.60510656526031</x>
        <y>96.23183516652448</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>257</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>51.39949089302518</x>
        <y>181.73639564248649</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>258</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>140.61625635482116</x>
        <y>118.88528060437326</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>259</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>81.11235868256772</x>
        <y>71.16703221921186</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>260</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>106.30383323544051</x>
        <y>47.66664077552125</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>261</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>78.36392430481997</x>
        <y>145.12300889005226</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>262</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>159.2795584664916</x>
        <y>175.42365153724947</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>263</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>89.12806123792214</x>
        <y>163.88074620615808</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>264</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>188.00194321004403</x>
        <y>167.99738752402368</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>265</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>186.544702443713</x>
        <y>156.12158395696437</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>266</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>2.8406717287810412</x>
        <y>38.380349570314664</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>267</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>28.76119804764801</x>
        <y>168.10637626762275</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>268</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>190.1115654346047</x>
        <y>36.24498374070968</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>269</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>67.43484884843447</x>
        <y>118.69086680825731</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>270</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>50.7959234692023</x>
        <y>165.04960719272802</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>271</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>50.75575271798458</x>
        <y>144.55723570362358</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>272</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>128.94257968083764</x>
        <y>47.32490472068322</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>273</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>1.9193343221312942</x>
        <y>112.82658785936086</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>274</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>51.82023785974064</x>
        <y>148.28034473499338</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>275</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>98.55603081185546</x>
        <y>178.02566808501155</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>276</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>143.40572664084078</x>
        <y>183.27302398341982</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>277</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>141.0619733646239</x>
        <y>54.122738136324955</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>278</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>162.29942916334053</x>
        <y>67.55247227617933</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>279</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>185.29686875363197</x>
        <y>118.81126461905944</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>280</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>104.06690998854779</x>
        <y>5.511220514887172</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>281</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>181.82429252421596</x>
        <y>47.0700962247878</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>282</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>17.604228655775245</x>
        <y>96.78328313290936</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>283</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>60.74231839144737</x>
        <y>136.8041677309854</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>284</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>158.57791000335715</x>
        <y>90.16811700419458</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>285</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>26.040369760119475</x>
        <y>115.48428978095157</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>286</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>155.33825554510548</x>
        <y>122.61758874267335</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>287</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>171.04754637183774</x>
        <y>49.44780022857469</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>288</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>130.8601631228818</x>
        <y>115.38443324744807</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>289</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>138.51066172926747</x>
        <y>85.05731662406394</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>290</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>17.01860916014879</x>
        <y>85.74431631403492</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>291</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>63.632763185894994</x>
        <y>73.28193598294403</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>292</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>145.51992557600448</x>
        <y>190.19645657449914</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>293</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>141.3669225587347</x>
        <y>128.37798094188392</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>294</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>5.624436269305666</x>
        <y>10.321359475496084</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>295</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>32.62210104212715</x>
        <y>80.99365929301005</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>296</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>146.82462375206796</x>
        <y>189.00512676494264</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>297</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>96.89937776974169</x>
        <y>61.868257009019125</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>298</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>71.17532023107206</x>
        <y>32.87953533289934</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>299</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>37.21430187397199</x>
        <y>21.880189704400976</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>300</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>123.17178528937387</x>
        <y>23.802492560334287</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>301</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>139.8264101859233</x>
        <y>106.93416114169838</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>302</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>34.97792194896952</x>
        <y>182.11554692137054</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>303</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>156.016327676095</x>
        <y>83.35423896139108</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>304</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>33.33352678715003</x>
        <y>148.24111721535743</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>305</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>22.525965053498552</x>
        <y>94.23130431241577</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>306</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>145.40586007739483</x>
        <y>194.77296443866655</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>307</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>64.78487058910738</x>
        <y>34.59908782949142</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>308</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>59.86114357954142</x>
        <y>143.29623794752337</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>309</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>133.03711837762597</x>
        <y>0.29677881350260726</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>310</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>184.05437940334514</x>
        <y>80.34917749334691</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>311</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>43.18494391360306</x>
        <y>0.7070568470557648</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>312</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>87.88579984985796</x>
        <y>183.6845166360299</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>313</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>35.679149788796785</x>
        <y>59.900754257451624</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>314</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>10.837936713278706</x>
        <y>68.65555543408139</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>315</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>28.088726188466005</x>
        <y>66.6117410256945</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>316</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>24.15305881985441</x>
        <y>127.36722357863377</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>317</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>167.92463302345024</x>
        <y>95.32246240241238</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>318</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>150.07537910034364</x>
        <y>189.29680149689028</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>319</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>36.60193942102408</x>
        <y>4.850860928459388</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>320</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>131.4804357164766</x>
        <y>107.37981261172979</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>321</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>4.702056299253576</x>
        <y>145.3571381517292</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>322</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>76.83393335740108</x>
        <y>40.92489706855471</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>323</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>81.41902810900726</x>
        <y>59.67311069186907</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>324</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>157.9286881274713</x>
        <y>35.89390157980119</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>325</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>172.59261547136273</x>
        <y>162.21173966194792</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>326</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>63.77983240079481</x>
        <y>110.66181735649987</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>327</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>94.27259269172448</x>
        <y>102.22907067171563</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>328</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>87.82927755519094</x>
        <y>154.83172363042706</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>329</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>61.424763006386954</x>
        <y>73.54903523332621</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>330</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>193.50994444846046</x>
        <y>100.37438735826956</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>331</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>160.15646353486886</x>
        <y>197.56265442862397</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>332</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>33.41179410209567</x>
        <y>85.51716211010236</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>333</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>39.55932830334419</x>
        <y>79.79114070992594</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>334</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>29.81531743952457</x>
        <y>106.52370973616816</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>335</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>145.66591758403607</x>
        <y>93.84627277397392</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>336</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>121.12689025085304</x>
        <y>141.7616054105135</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>337</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>31.696932817539736</x>
        <y>73.39512842700171</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>338</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>73.55413175311341</x>
        <y>184.10063535264334</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>339</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>179.21266954315877</x>
        <y>157.77936426661222</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>340</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>5.861701400590658</x>
        <y>176.44679868441557</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>341</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>164.32858198885157</x>
        <y>127.1649251930171</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>342</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>104.78813885934602</x>
        <y>2.6978015525372934</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>343</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>9.036210189825722</x>
        <y>37.29651951364714</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>344</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>69.09633192708777</x>
        <y>131.08113653458605</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>345</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>23.989325789242024</x>
        <y>102.76529191595212</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>346</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>32.563144827068456</x>
        <y>174.05783874991164</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>347</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>26.84717981820497</x>
        <y>33.708035418260465</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>348</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>23.651571274100892</x>
        <y>150.9696150146202</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>349</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>7.263970263317554</x>
        <y>178.4551746541966</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>350</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>137.19080046610807</x>
        <y>195.47642758858956</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>351</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>100.60258790901588</x>
        <y>10.122226122279043</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>352</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>158.02955707421086</x>
        <y>15.552042272281575</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>353</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>69.03288347183658</x>
        <y>86.65939835169405</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>354</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>197.11534487465465</x>
        <y>12.227887489891408</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>355</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>109.32664513099861</x>
        <y>51.47545505189106</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>356</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>7.450618560323541</x>
        <y>114.12792666368863</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>357</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>96.73499758330652</x>
        <y>87.34903664585806</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>358</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>143.42568088659735</x>
        <y>154.7201550387036</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>359</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>16.055115327242596</x>
        <y>23.72235108907279</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>360</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>36.54118641672248</x>
        <y>71.60060131854802</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>361</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>153.4641555525212</x>
        <y>182.30266241969497</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>362</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>179.79961054026052</x>
        <y>52.374917374947486</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>363</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>51.89511924887278</x>
        <y>55.715278818289924</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>364</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>160.22911392558143</x>
        <y>197.03727711739174</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>365</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>141.50401865962175</x>
        <y>198.17183635084353</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>366</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>144.1436553724308</x>
        <y>66.26788567722302</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>367</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>114.85639238072662</x>
        <y>187.02866922391485</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>368</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>14.754418142216963</x>
        <y>189.75568091879705</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>369</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>40.445861020644756</x>
        <y>132.87622199437286</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>370</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>70.41846318684537</x>
        <y>16.353121961748673</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>371</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>11.524098377391411</x>
        <y>188.46037552576104</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>372</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>19.80490164758435</x>
        <y>193.31200682922997</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>373</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>170.32256510777594</x>
        <y>170.204813941954</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>374</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>35.11638379627671</x>
        <y>106.63452905636245</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>375</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>87.80726944100199</x>
        <y>69.16884374165753</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>376</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>45.63781115512378</x>
        <y>137.1790704392972</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>377</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>120.23124867416645</x>
        <y>21.60172442725463</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>378</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>42.584241021086264</x>
        <y>172.9365035614701</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>379</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>106.9111221907689</x>
        <y>35.38132573733432</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>380</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>115.65012523180343</x>
        <y>149.06748739273075</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>381</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>66.70597176653999</x>
        <y>151.96624665556067</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>382</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>116.86108695969573</x>
        <y>92.96503821223025</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>383</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>138.3454858274232</x>
        <y>60.335069940591254</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>384</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>35.21169262537829</x>
        <y>57.75683948274251</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>385</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>183.64109836494706</x>
        <y>187.89865943504947</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>386</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>124.92709985349823</x>
        <y>7.139364140447135</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>387</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>121.19563498360651</x>
        <y>163.5898829983739</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>388</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>141.67701483198246</x>
        <y>36.967824799613645</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>389</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>116.85490603618803</x>
        <y>192.1746914581395</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>390</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>157.23646122348265</x>
        <y>101.21354943885676</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>391</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>58.04596641555313</x>
        <y>180.0770488919343</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>392</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>118.342960728573</x>
        <y>65.22048911724025</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>393</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>61.42203823259405</x>
        <y>117.333210601775</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>394</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>136.13358028390385</x>
        <y>97.99627507346685</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>395</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>82.86089860898949</x>
        <y>25.22615052347874</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>396</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>6.721379593879373</x>
        <y>94.78763681182285</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>397</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>38.69266957378596</x>
        <y>24.113141554020046</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>398</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>111.22857295130957</x>
        <y>95.51634459331788</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>399</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>126.7915415569141</x>
        <y>32.23798771734878</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>400</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>186.84699461455236</x>
        <y>34.76662976923288</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>401</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>122.45497909139493</x>
        <y>167.0773654715768</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>402</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
    <mote>
      <interface_config>
        org.contikios.cooja.interfaces.Position
        <x>33.03449972907999</x>
        <y>172.77024458531486</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiMoteID
        <id>403</id>
      </interface_config>
      <interface_config>
        org.contikios.cooja.contikimote.interfaces.ContikiRadio
        <bitrate>250.0</bitrate>
      </interface_config>
      <motetype_identifier>mtype332</motetype_identifier>
    </mote>
  </simulation>
  <plugin>
    org.contikios.cooja.plugins.SimControl
    <width>280</width>
    <z>3</z>
    <height>160</height>
    <location_x>400</location_x>
    <location_y>0</location_y>
  </plugin>
  <plugin>
    org.contikios.cooja.plugins.Visualizer
    <plugin_config>
      <skin>org.contikios.cooja.plugins.skins.IDVisualizerSkin</skin>
      <skin>org.contikios.cooja.plugins.skins.GridVisualizerSkin</skin>
      <skin>org.contikios.cooja.plugins.skins.MoteTypeVisualizerSkin</skin>
      <skin>org.contikios.cooja.plugins.skins.UDGMVisualizerSkin</skin>
      <viewport>1.1671649566739442 0.0 0.0 1.1671649566739442 66.21348020552065 40.83199757586018</viewport>
    </plugin_config>
    <width>400</width>
    <z>2</z>
    <height>400</height>
    <location_x>1</location_x>
    <location_y>1</location_y>
  </plugin>
  <plugin>
    org.contikios.cooja.plugins.LogListener
    <plugin_config>
      <filter />
      <formatted_time />
      <coloring />
    </plugin_config>
    <width>1184</width>
    <z>0</z>
    <height>240</height>
    <location_x>402</location_x>
    <location_y>162</location_y>
  </plugin>
  <plugin>
    org.contikios.cooja.plugins.Notes
    <plugin_config>
      <notes>Enter notes here</notes>
      <decorations>true</decorations>
    </plugin_config>
    <width>904</width>
    <z>4</z>
    <height>160</height>
    <location_x>680</location_x>
    <location_y>0</location_y>
  </plugin>
  <plugin>
    org.contikios.cooja.plugins.ScriptRunner
    <plugin_config>
      <script>GENERATE_MSG(0000000, "add-sink");&#xD;
//GENERATE_MSG(1000000, "remove-sink");&#xD;
//GENERATE_MSG(1020000, "add-sink");&#xD;
&#xD;
lostMsgs = 0;&#xD;
&#xD;
TIMEOUT(1000000, if(lastMsg != -1 &amp;&amp; lostMsgs == 0) { log.testOK(); } );&#xD;
&#xD;
lastMsg = -1;&#xD;
packets = "_________";&#xD;
hops = 0;&#xD;
&#xD;
while(true) {&#xD;
    YIELD();&#xD;
    if(msg.equals("remove-sink")) {&#xD;
        m = sim.getMoteWithID(3);&#xD;
        sim.removeMote(m);&#xD;
        log.log("removed sink\n");&#xD;
    } else if(msg.equals("add-sink")) {&#xD;
        if(!sim.getMoteWithID(3)) {&#xD;
            m = sim.getMoteTypes()[1].generateMote(sim);&#xD;
            m.getInterfaces().getMoteID().setMoteID(3);&#xD;
            sim.addMote(m);&#xD;
            log.log("added sink\n");&#xD;
         } else {&#xD;
            log.log("did not add sink as it was already there\n");      &#xD;
         }&#xD;
    } else if(msg.startsWith("Sending")) {&#xD;
        hops = 0;&#xD;
    } else if(msg.startsWith("#L") &amp;&amp; msg.endsWith("1; red")) {&#xD;
        hops++;&#xD;
    } else if(msg.startsWith("Data")) {&#xD;
//        log.log("" + msg + "\n");    &#xD;
        data = msg.split(" ");&#xD;
        num = parseInt(data[14]);&#xD;
        packets = packets.substr(0, num) + "*";&#xD;
        log.log("" + hops + " " + packets + "\n");&#xD;
//        log.log("Num " + num + "\n");&#xD;
        if(lastMsg != -1) {&#xD;
          if(num != lastMsg + 1) {&#xD;
            numMissed = num - lastMsg - 1;&#xD;
            lostMsgs += numMissed;&#xD;
            log.log("Missed messages " + numMissed + " before " + num + "\n");            &#xD;
            for(i = 0; i &lt; numMissed; i++) {&#xD;
                packets = packets.substr(0, lastMsg + i) + "_";    &#xD;
            }&#xD;
          }    &#xD;
        }&#xD;
        lastMsg = num;&#xD;
    }&#xD;
}</script>
      <active>true</active>
    </plugin_config>
    <width>962</width>
    <z>1</z>
    <height>596</height>
    <location_x>603</location_x>
    <location_y>43</location_y>
  </plugin>
</simconf>

