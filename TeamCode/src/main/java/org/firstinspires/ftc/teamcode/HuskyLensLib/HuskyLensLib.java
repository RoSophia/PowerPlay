package org.firstinspires.ftc.teamcode.HuskyLensLib;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.List;
import java.util.Vector;

/// THIS SHIT IS NOT USEFUL, KILL MYSELF
@I2cDeviceType()
@DeviceProperties(name = "Husky", description = "Husky cam", xmlTag = "HUSKY_CAM")
@Disabled
public class HuskyLensLib extends I2cDeviceSynchDevice<I2cDeviceSynch> { // I have become a husk of myself
    boolean checkOnceAgain;
    Vector<Byte> lastCmdSent;
    Telemetry telemetry;

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

    private final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x32);

    public Vector<Byte> MaSinucid() {
        Vector<Byte> b = new Vector<>();
        for (int i = 0; i < 255; ++i) {
            b.add(this.deviceClient.read(i, 1)[0]);
        }
        return b;
    }

    public void AddTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public HuskyLensLib(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        //address = addr;
        checkOnceAgain = true;
        lastCmdSent = new Vector<>();

        this.deviceClient.engage();
    }

    public void writeToHuskyLens(Vector<Byte> cmd) {
        cmd.add((byte) 0x00);
        telemetry.addData("Write len", cmd.size());
        lastCmdSent = cmd;
        byte[] barr = new byte[cmd.size()];
        for (int i = 0; i < cmd.size(); ++i) {
            barr[i] = cmd.get(i);
            telemetry.addData("i", (int)barr[i]);
        }
        this.deviceClient.write(12, barr);
    }

    static class SplitCommand {
        List<Byte> headers;
        Byte address;
        int data_length;
        Byte command;
        List<Byte> data;
        int checkSum;
    }

    void printSplitCommand(SplitCommand spcl) {
        telemetry.addLine("   \\-Split");
        telemetry.addData("    -Haed", spcl.headers);
        telemetry.addData("    -Addr", spcl.address);
        telemetry.addData("    -datl", spcl.data_length);
        telemetry.addData("    -comm", spcl.command);
        telemetry.addData("    -data", spcl.data);
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

        printSplitCommand(sc);
        return sc;
    }

    public SplitCommand getCommandSplit() {
        telemetry.addLine(" \\-GetCommandSplit: Start");
        telemetry.addLine("U");
        //telemetry.update();
        Vector<Byte> byteString = new Vector<>();
        for (int i = 0; i < 5; ++i) {
            byteString.add(this.deviceClient.read8());
        }
        telemetry.addData("  -GetCommandSplit: bstr", byteString);
        telemetry.addLine("I");
        //telemetry.update();
        for (int i = 0; i < byteString.elementAt(3) + 1; ++i) {
            byteString.add(this.deviceClient.read8());
        }
        telemetry.addData("  -GetCommandSplit: bst2", byteString);
        telemetry.addLine("P");
        //telemetry.update();

        return splitCommandToParts(byteString);
    }

    public int nidLearned; // Number of ID learned
    public int fNumber;    // Frame number

    public Vector<EElement> processReturnData() {
        telemetry.addLine("ProcessReturnData: Start");
        telemetry.addLine("1");
        //telemetry.update();

        try {
            telemetry.addLine("C");
            //telemetry.update();
            SplitCommand spc = getCommandSplit();
            telemetry.addLine("2");
            //telemetry.update();
            if (spc.command == 0x2E) {
                checkOnceAgain = true;
                telemetry.addLine("ProcessReturnData: Comand iz 0x2E. Knoc knoc");
                telemetry.addLine("3");
                //telemetry.update();
                return new Vector<>();
            } else {
                telemetry.addLine("4");
                //telemetry.update();
                Vector<List<Byte>> returnData = new Vector<>();
                int numberOfBlocksOrArrow = spc.data.get(0) + spc.data.get(1) << 8;
                int numberOfIDLearned = spc.data.get(2) + spc.data.get(3) << 8;
                int frameNumber = spc.data.get(4) + spc.data.get(5) << 8;
                telemetry.addData("ProcessReturnData: nrboa", numberOfBlocksOrArrow);
                telemetry.addData("ProcessReturnData: nridl", numberOfIDLearned);
                telemetry.addData("ProcessReturnData: nrfrn", frameNumber);
                telemetry.addLine("5");
                //telemetry.update();
                boolean isBlock = true;
                for (int i = 0; i < numberOfBlocksOrArrow; ++i) {
                    SplitCommand tspc = getCommandSplit();
                    isBlock = tspc.command == 0x2A; // ??????????????????????????
                    returnData.add(tspc.data);
                }
                telemetry.addLine("6");
                //telemetry.update();

                telemetry.addLine("ProcessReturnData: preinit");
                Vector<Vector<Integer>> finalData = new Vector<>();
                Vector<Integer> tmp = new Vector<>();
                telemetry.addLine("ProcessReturnData: postinit");
                for (List<Byte> i : returnData) {
                    telemetry.addLine("ProcessReturnData: clear");
                    tmp.clear();
                    telemetry.addData("ProcessReturnData: i", i);
                    for (int q = 0; q < i.size(); q += 4) {
                        int low = i.get(0);
                        int high = i.get(1);
                        int val;
                        if (high > 0) {
                            val = low + 255 + high;
                        } else {
                            val = low;
                        }
                        tmp.add(val);
                    }

                    finalData.add(tmp);
                    telemetry.addData("ProcessReturnData: fd", finalData);
                }

                telemetry.addLine("7");
                //telemetry.update();
                checkOnceAgain = true;
                Vector<EElement> ret = convert_to_class_object(finalData, isBlock);
                nidLearned = numberOfIDLearned;
                fNumber = frameNumber;
                return ret;
            }
        } catch (Exception E) {
            telemetry.addData("ProcessReturnData: Excieption", E.getMessage());
            telemetry.addData("", Arrays.toString(E.getStackTrace()));
            if (checkOnceAgain) {
                // huskylensSer.timeout = 5; TODO: I DO NOT GET IT??
                checkOnceAgain = false;
                // huskylensSer.timeout = .5;
                return processReturnData();
            }
            /// PRINT("Read response error, please try again");
            /*
            huskylensSer.flushInput();
            huskylensSer.flushOutput();
            huskylensSer.flush();
             */
            return new Vector<>();
        }
    }

    public Vector<EElement> convert_to_class_object(Vector<Vector<Integer>> data, boolean isBlock) {
        Vector<EElement> tmp = new Vector<>();
        for (Vector<Integer> i : data) {
            EElement obj;
            obj = new EElement(i.get(0), i.get(1), i.get(2), i.get(3), i.get(4), isBlock);
            tmp.add(obj);
        }

        return tmp;
    }

    Byte[] commandHeaderAndAddress = {(byte)0x55, (byte) 0xAA, (byte)0x11};

    public int knock() {
        Vector<Byte> cmd = new Vector<>(Arrays.asList(commandHeaderAndAddress));
        Byte[] kms = {0x00, 0x2c, 0x3c};
        cmd.addAll(Arrays.asList(kms));
        writeToHuskyLens(cmd);
        Vector<EElement> x = processReturnData();
        return x.size();
    }

    byte calculateChecksum(Vector<Byte> hexStr) {
        int total = 0;
        for (int i = 0; i < hexStr.size(); ++i) {
            total += hexStr.elementAt(i);
        }
        return (byte) (total & 0xFF);
    }

    public int learn(int x) {
        Byte[] data = {(byte) (x & 0xFF), (byte) ((x & 0xFF00) >> 2)};
        Byte[] dataLen = {0x02};
        Vector<Byte> cmd = new Vector<>(Arrays.asList(data));
        cmd.addAll(Arrays.asList(commandHeaderAndAddress));
        cmd.addAll(Arrays.asList(dataLen));
        cmd.add((byte) 0x36);
        cmd.addAll(Arrays.asList(data));
        cmd.add(calculateChecksum(cmd));
        writeToHuskyLens(cmd);
        return knock();//processReturnData();
    }

    public int forget() {
        Vector<Byte> cmd = new Vector<>(Arrays.asList(commandHeaderAndAddress));
        Byte[] st = {(byte) 0x00, (byte) 0x37, (byte) 0x47};
        cmd.addAll(Arrays.asList(st));
        writeToHuskyLens(cmd);
        return knock();//self.processReturnData();
    }

    private Vector<Byte> atov(byte[] bs) {
        Vector<Byte> b = new Vector<>();
        for (byte bi : bs) {
            b.add(bi);
        }
        return b;
    }

    public int setCustomName(String name, int idV) {
        byte nameDataSize = (byte) (name.length() + 1);
        byte[] bs = name.getBytes(StandardCharsets.UTF_8);
        Vector<Byte> b = atov(bs);
        b.add((byte) 0x00);
        byte localId = (byte) idV;
        Vector<Byte> data = new Vector<>();
        data.add(localId);
        data.add(nameDataSize);
        data.addAll(b);
        byte dataLen = (byte) data.size();
        Vector<Byte> cmd = new Vector<>(Arrays.asList(commandHeaderAndAddress));
        cmd.add(dataLen);
        cmd.add((byte) 0x2f);
        cmd.addAll(data);
        cmd.add(calculateChecksum(cmd));
        writeToHuskyLens(cmd);
        return knock();
    }

    public int customText(String name, int xV, int yV) {
        byte nameDataSize = (byte) name.length();
        Vector<Byte> x = new Vector<>();
        if (xV > 255) {
            x.add((byte) 0xFF);
            x.add((byte) (xV & 0xFF));
        } else {
            x.add((byte) 0x00);
            x.add((byte) xV);
        }

        Vector<Byte> b = atov(name.getBytes(StandardCharsets.UTF_8));

        Vector<Byte> data = new Vector<>();
        data.add(nameDataSize);
        data.addAll(x);
        data.add((byte) yV);
        data.addAll(b);
        byte dataLen = (byte) data.size();
        Vector<Byte> cmd = new Vector<>(Arrays.asList(commandHeaderAndAddress));
        cmd.add(dataLen);
        cmd.add((byte) 0x34);
        cmd.addAll(data);
        cmd.add(calculateChecksum(cmd));
        writeToHuskyLens(cmd);
        return knock();
    }

    public int clearText() {
        Vector<Byte> cmd = new Vector<>(Arrays.asList(commandHeaderAndAddress));
        byte[] bs = {(byte) 0x00, (byte) 0x35, (byte) 0x45};
        Vector<Byte> b = atov(bs);
        cmd.addAll(b);
        writeToHuskyLens(cmd);
        return knock();
    }

    public Vector<EElement> requestAll() {
        Vector<Byte> cmd = new Vector<>(Arrays.asList(commandHeaderAndAddress));
        byte[] bs = {(byte) 0x00, (byte) 0x20, (byte) 0x30};
        Vector<Byte> b = atov(bs);
        cmd.addAll(b);
        writeToHuskyLens(cmd);
        return processReturnData();
    }

    /*

        def saveModelToSDCard(self,idVal):
            idVal = "{:04x}".format(idVal)
            idVal = idVal[2:]+idVal[0:2]
            cmd = commandHeaderAndAddress+"0232"+idVal
            cmd += self.calculateChecksum(cmd)
            cmd = self.cmdToBytes(cmd)
            self.writeToHuskyLens(cmd)
            return self.processReturnData()

        def loadModelFromSDCard(self,idVal):
            idVal = "{:04x}".format(idVal)
            idVal = idVal[2:]+idVal[0:2]
            cmd = commandHeaderAndAddress+"0233"+idVal
            cmd += self.calculateChecksum(cmd)
            cmd = self.cmdToBytes(cmd)
            self.writeToHuskyLens(cmd)
            return self.processReturnData()

        def savePictureToSDCard(self):
            self.huskylensSer.timeout=5
            cmd = self.cmdToBytes(commandHeaderAndAddress+"003040")
            self.writeToHuskyLens(cmd)
            return self.processReturnData()

        def saveScreenshotToSDCard(self):
            cmd = self.cmdToBytes(commandHeaderAndAddress+"003949")
            self.writeToHuskyLens(cmd)
            return self.processReturnData()

     */

    public Vector<EElement> blocks() {
        Vector<Byte> cmd = new Vector<>(Arrays.asList(commandHeaderAndAddress));
        byte[] bs = {(byte) 0x00, (byte) 0x21, (byte) 0x31};
        Vector<Byte> b = atov(bs);
        cmd.addAll(b);
        writeToHuskyLens(cmd);
        return processReturnData();
    }

    public Vector<EElement> arrows() {
        Vector<Byte> cmd = new Vector<>(Arrays.asList(commandHeaderAndAddress));
        byte[] bs = {(byte) 0x00, (byte) 0x22, (byte) 0x32};
        Vector<Byte> b = atov(bs);
        writeToHuskyLens(cmd);
        cmd.addAll(b);
        return processReturnData();
    }

    public Vector<EElement> learned() {
        Vector<Byte> cmd = new Vector<>(Arrays.asList(commandHeaderAndAddress));
        byte[] bs = {(byte) 0x00, (byte) 0x23, (byte) 0x33};
        Vector<Byte> b = atov(bs);
        writeToHuskyLens(cmd);
        cmd.addAll(b);
        return processReturnData();
    }

    public Vector<EElement> learnedBlocks() {
        Vector<Byte> cmd = new Vector<>(Arrays.asList(commandHeaderAndAddress));
        byte[] bs = {(byte) 0x00, (byte) 0x24, (byte) 0x34};
        Vector<Byte> b = atov(bs);
        writeToHuskyLens(cmd);
        cmd.addAll(b);
        return processReturnData();
    }

    public Vector<EElement> learnedArrows() {
        Vector<Byte> cmd = new Vector<>(Arrays.asList(commandHeaderAndAddress));
        byte[] bs = {(byte) 0x00, (byte) 0x25, (byte) 0x35};
        Vector<Byte> b = atov(bs);
        writeToHuskyLens(cmd);
        cmd.addAll(b);
        return processReturnData();
    }

    public Vector<EElement> getObjectByID(int idVal) {
        Vector<Byte> cmd = new Vector<>(Arrays.asList(commandHeaderAndAddress));
        cmd.add((byte) 0x02);
        cmd.add((byte) 0x26);
        cmd.add((byte) (idVal & 0xFF));
        cmd.add((byte) ((idVal & 0xFF00) >> 2));
        cmd.add(calculateChecksum(cmd));
        writeToHuskyLens(cmd);
        return processReturnData();
    }

    public Vector<EElement> getBlocksByID(int idVal) {
        Vector<Byte> cmd = new Vector<>(Arrays.asList(commandHeaderAndAddress));
        cmd.add((byte) 0x02);
        cmd.add((byte) 0x27);
        cmd.add((byte) (idVal & 0xFF));
        cmd.add((byte) ((idVal & 0xFF00) >> 2));
        cmd.add(calculateChecksum(cmd));
        writeToHuskyLens(cmd);
        return processReturnData();
    }

    public Vector<EElement> getArrowsByID(int idVal) {
        Vector<Byte> cmd = new Vector<>(Arrays.asList(commandHeaderAndAddress));
        cmd.add((byte) 0x02);
        cmd.add((byte) 0x28);
        cmd.add((byte) (idVal & 0xFF));
        cmd.add((byte) ((idVal & 0xFF00) >> 2));
        cmd.add(calculateChecksum(cmd));
        writeToHuskyLens(cmd);
        return processReturnData();
    }

    public int algorithm(byte alg) {
        Vector<Byte> cmd = new Vector<>(Arrays.asList(commandHeaderAndAddress));
        cmd.add((byte) 0x02);
        cmd.add((byte) 0x2d);
        cmd.add(alg);
        cmd.add((byte) 0x00);
        cmd.add(calculateChecksum(cmd));
        writeToHuskyLens(cmd);
        return knock();
    }

}
