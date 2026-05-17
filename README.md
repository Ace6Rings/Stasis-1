# OVERVIEW

The Reshular Robot Arm is a 6 DOF robotic arm. It is operated over bluetooth so it can be operated remotely.
5 stepper motors, using the TMC2209 driver, and 1 servo motor is used to make this huge robotic arm.

# Purpose

I made this because I've always been fascinated on how to use technology to make an impact on the physical world. 
I've seen small robotic arms powered with servo motors. But I noticed that scaling it up in size changes the materials I need 
and introduces (newish) problems like insufficient torque when using a servo motor. These robotic arms are also shorter ranged and can only hold
very light and small items. Thats why I built this robotic arm. I can lift items that are impossible for small robotic arms like a big book. 
The most important thing is that I became way better at eda since this is my first project that uses a smd PCB.

# Schematic
<img width="1082" height="624" alt="schematic 2" src="https://github.com/user-attachments/assets/3fca7e48-12bf-4a3e-8217-b0f137b467fb" />
<img width="1459" height="841" alt="scematic1" src="https://github.com/user-attachments/assets/473986a7-882b-485c-b744-1604b7c948b0" />
There are 5 tmc 2209 motor drivers. They are almost identical. Only their gpio assignments to dir and step changes and also their uart configuration.

# PCB
<img width="1098" height="803" alt="pcb" src="https://github.com/user-attachments/assets/0e9a171d-74e7-4a03-845d-950562f76a62" />
The PCB features an esp32, 5 tmc 2209 motor drivers, 1 servo motor screw connector.
This is done on a 2 layer board for the maxmimum speed of building and cost efficiency.
# Model
The purpose of the model is not to make it look good but to make it as light as possible to test in real life. 
It features 6 degrees of freedom, which could move (almost) ny item in any way you could think of.
<img width="1616" height="791" alt="model2" src="https://github.com/user-attachments/assets/5baa0a8e-6e98-479f-94c3-4647e9544fd1" />
<img width="1614" height="783" alt="model1" src="https://github.com/user-attachments/assets/bed8afed-dd94-4264-a4e6-8dd0c09be38b" />
The base has an hexagonal pattern which maximizes strength, aesthetics, and filament efficiency.

# Testing

N/A (will include after getting parts)



# Bill of Materials

| Name                          | Purpose                            | Quantity | Total Cost (USD) | Link                                                                                                                                                                                         | Distributor |
|-------------------------------|------------------------------------|----------|------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------|
| li ion batteries              | batteries to power my robot arm    | 1        | 25.98            | https://www.amazon.com/Mupoer-Rechargeable-Batteries-Integrated-Charging/dp/B0D4M581B3/ref=sr_1_15?sr=8-15                                                                                   | amazon      |
| ds3225                        | gripper motor                      | 1        | 16.99            | https://www.amazon.com/gp/product/B0CNTG877G/ref=ox_sc_act_title_1?smid=A1PKC2PUMNR8VD                                                                                                       | amazon      |
| HEAT SET INSERTS              | To assemble my robot arm           | 1        | 9.98             | https://www.amazon.com/M2-M2-5-M3-Threaded-Inserts/dp/B0FH4JQ3VB/ref=sr_1_4?sr=8-4                                                                                                           | amazon      |
| 175-TMC2209-LA-TCT-ND         | IC MTR DRV 4.75-28V QFN28          | 10       | 40.89            | https://www.digikey.com/en/products/detail/analog-devices-inc-maxim-integrated/TMC2209-LA-T/10232491                                                                                         | DIGIKEY     |
| NCP1117LPST33T3GOSCT-ND       | IC REG LINEAR 3.3V 1A SOT-223      | 10       | 2.62             | https://www.digikey.com/en/products/detail/onsemi/NCP1117LPST33T3G/1967376                                                                                                                   | DIGIKEY     |
| 296-50511-1-ND                | UNIDIR PRECISION SURGE DIODE       | 10       | 7.06             | https://www.digikey.com/en/products/detail/texas-instruments/TVS1400DRVR/11589096                                                                                                            | DIGIKEY     |
| 2073-USB4085-GF-ACT-ND        | CONN RCPT USB2.0 TYPE C 16+8POS    | 2        | 2.15             | https://www.digikey.com/en/products/detail/gct/USB4085-GF-A/9859704                                                                                                                          | DIGIKEY     |
| CKN10285CT-ND                 | SWITCH TACTILE SPST-NO 0.05A 32V   | 10       | 5.81             | https://www.digikey.com/en/products/detail/c-k/KMR241NG-ULC-LFS/2747141                                                                                                                      | DIGIKEY     |
| 490-10700-1-ND                | CAP CER 0.1UF 50V X7R 0402         | 50       | 0.81             | https://www.digikey.com/en/products/detail/murata-electronics/GRM155R71H104KE14D/4358653                                                                                                     | DIGIKEY     |
| 277-1722-ND                   | TERM BLK 3POS SIDE ENT 3.5MM PCB   | 1        | 2.10             | https://www.digikey.com/en/products/detail/phoenix-contact/1984620/950937                                                                                                                    | DIGIKEY     |
| 1276-1096-1-ND                | CAP CER 10UF 16V X5R 0805          | 10       | 0.54             | https://www.digikey.com/en/products/detail/samsung-electro-mechanics/CL21A106KOQNNNE/3886902                                                                                                 | DIGIKEY     |
| 273-KDV12DR100ETCT-ND         | RES 0.1 OHM 0.5% 1/2W 1206         | 25       | 3.00             | https://www.digikey.com/en/products/detail/ohmite/KDV12DR100ET/13691867                                                                                                                      | DIGIKEY     |
| 311-5.10KLRCT-ND              | RES 5.1K OHM 1% 1/16W 0402         | 25       | 0.37             | https://www.digikey.com/en/products/detail/yageo/RC0402FR-075K1L/726633                                                                                                                      | DIGIKEY     |
| 541-10.0KLCT-ND               | RES 10K OHM 1% 1/16W 0402          | 25       | 0.59             | https://www.digikey.com/en/products/detail/vishay-dale/CRCW040210K0FKED/1176095                                                                                                              | DIGIKEY     |
| 311-100KLRCT-ND               | RES 100K OHM 1% 1/16W 0402         | 25       | 0.37             | https://www.digikey.com/en/products/detail/yageo/RC0402FR-07100KL/726651                                                                                                                     | DIGIKEY     |
| 311-1.00KLRCT-ND              | RES 1K OHM 1% 1/16W 0402           | 25       | 0.37             | https://www.digikey.com/en/products/detail/yageo/RC0402FR-071KL/726544                                                                                                                       | DIGIKEY     |
| 277-1721-ND                   | TERM BLK 2POS SIDE ENT 3.5MM PCB   | 1        | 1.46             | https://www.digikey.com/en/products/detail/phoenix-contact/1984617/950934                                                                                                                    | DIGIKEY     |
| 490-4763-1-ND                 | CAP CER 0.022UF 50V X7R 0402       | 50       | 1.01             | https://www.digikey.com/en/products/detail/murata-electronics/GCM155R71H223KA55D/2756835                                                                                                     | DIGIKEY     |
| 277-1860-ND                   | TERM BLK 4POS SIDE ENT 3.5MM PCB   | 5        | 13.45            | https://www.digikey.com/en/products/detail/phoenix-contact/1984633/950950                                                                                                                    | DIGIKEY     |
| 1965-ESP32-S3-WROOM-1-N4CT-ND | RF TXRX MODULE BT PCB TRACE SMD    | 2        | 11.46            | https://www.digikey.com/en/products/detail/espressif-systems/ESP32-S3-WROOM-1-N4/16162639                                                                                                    | DIGIKEY     |
| JLCPCB                        | To put my components on            | 1        | 36.48            | https://trade.jlcpcb.com/checkout/payMethod?systemType=order_pcb&calType=PAY&batchNum=W2026042809527003&spm=Jlcpcb.Confirmorder.1001                                                         | jlcpcb      |
| NEMA 17 motor thin            | For the arms                       | 5        | 17.50            | https://www.alibaba.com/product-detail/Special-Offer-26mm-Length-NEMA-17_1601043669710.html?spm=a2700.prosearch.normal_offer.d_title.659167afAhymea&priceId=58dcbf79d3ef4e1099ae55c89d924238 | Alibaba     |
| M2-M5 assortment screws       | To screw pcb, battery holders, and | 1        | 12.38            | https://www.amazon.com/Countersunk-Assortment-Phillips-Machine-Washers/dp/B0F87JTJ6T/ref=sr_1_3?sr=8-3                                                                                       | Amazon      |
| mr105                         | bearings for high torque load      | 4        | 8.79             | https://www.amazon.com/MR105-2RS-Deep-Groove-Ball-Bearings/dp/B0DXZNSMJC/ref=sr_1_6?sr=8-6                                                                                                   | amazon      |
