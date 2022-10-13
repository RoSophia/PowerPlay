package org.firstinspires.ftc.teamcode.husk;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Vector;

import kotlin.collections.EmptyList;

class AlgoByteId {
    public String ALGORITHM_OBJECT_TRACKING = "0100";
    public String ALGORITHM_FACE_RECOGNITION = "0000";
    public String ALGORITHM_OBJECT_RECOGNITION = "0200";
    public String ALGORITHM_LINE_TRACKING = "0300";
    public String ALGORITHM_COLOR_RECOGNITION = "0400";
    public String ALGORITHM_TAG_RECOGNITION = "0500";
    public String ALGORITHM_OBJECT_CLASSIFICATION = "0600";
    public String ALGORITHM_QR_CODE_RECOGNTITION = "0700";
    public String ALGORITHM_BARCODE_RECOGNTITION = "0800";
}

class EElement {
    public int x0;
    public int y0;
    public int x1;
    public int y1;
    public int ID;
    public int learned;

    enum EType {ARROW, BLOCK}

    ;
    public EType type;

    public EElement(int xt, int yt, int xh, int yh, int id, boolean ct) {
        x0 = xt;
        y0 = yt;
        x1 = xh;
        y1 = yh;
        ID = id;
        learned = ID > 0 ? 1 : 0;
        type = ct ? EType.BLOCK : EType.ARROW;
    }
}

class HuskyLensLib {
    int address;
    boolean checkOnceAgain;
    Vector<Byte> lastCmdSent;

    public HuskyLensLib(String comPort, int addr) {
        address = addr;
        checkOnceAgain = true;
        lastCmdSent = new Vector<>();
    }

    public void writeToHuskyLens(Vector<Byte> cmd) {
        lastCmdSent = cmd;
        /// TODO: AAA
        huskylensSer.write_i2c_block_data(address, 12, cmd);
    }

    static class SplitCommand {
        List<Byte> headers;
        Byte address;
        int data_length;
        Byte command;
        List<Byte> data;
        int checkSum;
    }

    public SplitCommand splitCommandToParts(Vector<Byte> str) {
        SplitCommand sc = new SplitCommand();
        sc.headers = str.subList(0, 2);
        sc.address = str.elementAt(2);
        sc.data_length = str.elementAt(3);
        sc.command = str.elementAt(4);
        if (sc.data_length > 0) {
            sc.data = str.subList(5, 5 + sc.data_length);
        }
        sc.checkSum = str.elementAt(5 + sc.data_length);

        return sc;
    }

    public SplitCommand getCommandSplit() {
        Vector<Byte> byteString = new Vector<>();
        for (int i = 0; i < 5; ++i) {
            byteString.add(huskylensSer.read_byte(address));
        }
        for (int i = 0; i < byteString.elementAt(3) + 1; ++i) {
            byteString.add(huskylensSer.read_byte(address));
        }

        return splitCommandToParts(byteString);
    }

    public Vector<EElement> processReturnData(boolean numIdLearnFlag, boolean frameFlag) {
        Vector<Byte> byteString = new Vector<>();
        try {
            SplitCommand spc = getCommandSplit();
            if (spc.command == 0x2E) {
                checkOnceAgain = true;
                return "Knock Recieved";
            } else {
                Vector<List<Byte>> returnData = new Vector<>();
                int numberOfBlocksOrArrow = spc.data.get(0) + spc.data.get(1) << 8;
                int numberOfIDLearned = spc.data.get(2) + spc.data.get(3) << 8;
                int frameNumber = spc.data.get(4) + spc.data.get(5) << 8;
                boolean isBlock = true;
                for (int i = 0; i < numberOfBlocksOrArrow; ++i) {
                    SplitCommand tspc = getCommandSplit();
                    isBlock = tspc.command == 0x2A; // ??????????????????????????
                    returnData.add(tspc.data);
                }

                Vector<List<Integer>> finalData = new Vector<>();
                Vector<Integer> tmp = new Vector<>();
                for (List<Byte> i : returnData) {
                    tmp.clear();
                    for (int q = 0; q < i.size(); q += 4) {
                        int low = i.get(0);
                        int high = i.get(1);
                        int val = 0;
                        if (high > 0) {
                            val = low + 255 + high;
                        } else {
                            val = low;
                        }
                        tmp.add(val);
                    }

                    finalData.add(tmp);
                }

                checkOnceAgain = true;
                Vector<EElement> ret = convert_to_class_object(finalData, isBlock);
                /*if (numIdLearnFlag) { /// TODO: Fix
                    ret.add(numberOfIDLearned);
                }
                if (frameFlag) {
                    ret.add(frameNumber);
                }*/
                return ret;
            }
        } catch (Exception E) {
            if (checkOnceAgain) {
                // huskylensSer.timeout = 5; TODO: I DO NOT GET IT??
                checkOnceAgain = false;
                // huskylensSer.timeout = .5;
                return processReturnData(false, false);
            }
            /// PRINT("Read response error, please try again");
            huskylensSer.flushInput();
            huskylensSer.flushOutput();
            huskylensSer.flush();
            return new Vector<>();
        }
    }


    public Vector<EElement> convert_to_class_object(Vector<List<Integer>> data, boolean isBlock) {
        Vector<EElement> tmp = new Vector<>();
        for (List<Integer> i : data) {
            EElement obj;
            obj = new EElement(i.get(0), i.get(1), i.get(2), i.get(3), i.get(4), isBlock);
            tmp.add(obj);
        }

        return tmp;
    }

    Byte[] commandHeaderAndAddress = {0x55, (byte)0xAA, 0x11};

    public int knock() {
        Vector<Byte> cmd = new Vector<>(Arrays.asList(commandHeaderAndAddress));
        Byte[] kms = {0x00, 0x2c, 0x3c};
        cmd.addAll(Arrays.asList(kms));
        writeToHuskyLens(cmd);
        Vector<EElement> x = processReturnData(false, false);
        return x.size();
    }

    def learn(self, x):
    data ="{:04x}".

    format(x)

    part1=data[2:]
    part2=data[0:2]
            #
    reverse to
    correct endiness
    data=part1+
    part2
            dataLen = "{:02x}".format(len(data)//2)
            cmd = commandHeaderAndAddress + dataLen + "36" + data
            cmd += self.calculateChecksum(cmd)
            cmd = self.cmdToBytes(cmd)
            self.writeToHuskyLens(cmd)
        return self.processReturnData()

    def forget(self):
    cmd =self.cmdToBytes(commandHeaderAndAddress+"003747")
            self.writeToHuskyLens(cmd)
            return self.processReturnData()

    def setCustomName(self, name, idV):
    nameDataSize ="{:02x}".

    format(len(name)+1)
    name =name.encode("utf-8").

    hex()+"00"
    localId ="{:02x}".

    format(idV)

    data =localId+nameDataSize+
    name
            dataLen = "{:02x}".format(len(data)//2)
            cmd = commandHeaderAndAddress + dataLen + "2f" + data
            cmd += self.calculateChecksum(cmd)
            cmd = self.cmdToBytes(cmd)
            self.writeToHuskyLens(cmd)
        return self.processReturnData()

    def customText(self, nameV, xV, yV):
    name=nameV.encode("utf-8").

    hex()

    nameDataSize ="{:02x}".

    format(len(name)//2)
        if(xV>255):
    x="ff"+"{:02x}".

    format(xV%255)
        else:
    x="00"+"{:02x}".

    format(xV)

    y="{:02x}".

    format(yV)

    data =nameDataSize+x+y+
    name
            dataLen = "{:02x}".format(len(data)//2)

            cmd = commandHeaderAndAddress + dataLen + "34" + data
            cmd += self.calculateChecksum(cmd)
            cmd = self.cmdToBytes(cmd)
            self.writeToHuskyLens(cmd)
        return self.processReturnData()

    def clearText(self):
    cmd =self.cmdToBytes(commandHeaderAndAddress+"003545")
            self.writeToHuskyLens(cmd)
            return self.processReturnData()

    def requestAll(self):
    cmd =self.cmdToBytes(commandHeaderAndAddress+"002030")
            self.writeToHuskyLens(cmd)
            return self.processReturnData()

    def saveModelToSDCard(self, idVal):
    idVal ="{:04x}".

    format(idVal)

    idVal =idVal[2:]+idVal[0:2]
    cmd =commandHeaderAndAddress+"0232"+idVal
    cmd +=self.calculateChecksum(cmd)
    cmd =self.cmdToBytes(cmd)
            self.writeToHuskyLens(cmd)
            return self.processReturnData()

    def loadModelFromSDCard(self, idVal):
    idVal ="{:04x}".

    format(idVal)

    idVal =idVal[2:]+idVal[0:2]
    cmd =commandHeaderAndAddress+"0233"+idVal
    cmd +=self.calculateChecksum(cmd)
    cmd =self.cmdToBytes(cmd)
            self.writeToHuskyLens(cmd)
            return self.processReturnData()

    def savePictureToSDCard(self):
    self.huskylensSer.timeout=5
    cmd =self.cmdToBytes(commandHeaderAndAddress+"003040")
            self.writeToHuskyLens(cmd)
            return self.processReturnData()

    def saveScreenshotToSDCard(self):
    cmd =self.cmdToBytes(commandHeaderAndAddress+"003949")
            self.writeToHuskyLens(cmd)
            return self.processReturnData()

    def blocks(self):
    cmd =self.cmdToBytes(commandHeaderAndAddress+"002131")
            self.writeToHuskyLens(cmd)
            return self.processReturnData()[0]

    def arrows(self):
    cmd =self.cmdToBytes(commandHeaderAndAddress+"002232")
            self.writeToHuskyLens(cmd)
            return self.processReturnData()[0]

    def learned(self):
    cmd =self.cmdToBytes(commandHeaderAndAddress+"002333")
            self.writeToHuskyLens(cmd)
            return self.processReturnData()[0]

    def learnedBlocks(self):
    cmd =self.cmdToBytes(commandHeaderAndAddress+"002434")
            self.writeToHuskyLens(cmd)
            return self.processReturnData()[0]

    def learnedArrows(self):
    cmd =self.cmdToBytes(commandHeaderAndAddress+"002535")
            self.writeToHuskyLens(cmd)
            return self.processReturnData()[0]

    def getObjectByID(self, idVal):
    idVal ="{:04x}".

    format(idVal)

    idVal =idVal[2:]+idVal[0:2]
    cmd =commandHeaderAndAddress+"0226"+idVal
    cmd +=self.calculateChecksum(cmd)
    cmd =self.cmdToBytes(cmd)
            self.writeToHuskyLens(cmd)
            return self.processReturnData()[0]

    def getBlocksByID(self, idVal):
    idVal ="{:04x}".

    format(idVal)

    idVal =idVal[2:]+idVal[0:2]
    cmd =commandHeaderAndAddress+"0227"+idVal
    cmd +=self.calculateChecksum(cmd)
    cmd =self.cmdToBytes(cmd)
            self.writeToHuskyLens(cmd)
            return self.processReturnData()[0]

    def getArrowsByID(self, idVal):
    idVal ="{:04x}".

    format(idVal)

    idVal =idVal[2:]+idVal[0:2]
    cmd =commandHeaderAndAddress+"0228"+idVal
    cmd +=self.calculateChecksum(cmd)
    cmd =self.cmdToBytes(cmd)
            self.writeToHuskyLens(cmd)
            return self.processReturnData()[0]

    def algorthim(self, alg):
            if
    alg in
    algorthimsByteID:
    cmd =commandHeaderAndAddress+"022d"+algorthimsByteID[alg]
    cmd +=self.calculateChecksum(cmd)
    cmd =self.cmdToBytes(cmd)
            self.writeToHuskyLens(cmd)
            return self.processReturnData()
            else:

    print("INCORRECT ALGORITHIM NAME")

    def count(self):
    cmd =self.cmdToBytes(commandHeaderAndAddress+"002030")
            self.writeToHuskyLens(cmd)
            return

    len(self.processReturnData())

    def learnedObjCount(self):
    cmd =self.cmdToBytes(commandHeaderAndAddress+"002030")
            self.writeToHuskyLens(cmd)
            return self.processReturnData(numIdLearnFlag=True)[-1]

    def frameNumber(self):
    cmd =self.cmdToBytes(commandHeaderAndAddress+"002030")
            self.writeToHuskyLens(cmd)
            return self.processReturnData(frameFlag=True)[-1]

    @I2cDeviceType()
    @DeviceProperties(name = "QWIIC LED Stick", description = "Sparkfun QWIIC LED Stick", xmlTag = "QWIIC_LED_STICK")
    public class Husky extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {

        @Override
        public Manufacturer getManufacturer() {
            return Manufacturer.Other;
        }

        @Override
        protected synchronized boolean doInitialize() {
            return true;
        }

        @Override
        public String getDeviceName() {
            return "Huskylens";
        }

        private final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x23);

        public Husky(I2cDeviceSynchSimple deviceClient) {
            super(deviceClient, true);

            this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
            super.registerArmingStateCallback(false);
        }

    }
