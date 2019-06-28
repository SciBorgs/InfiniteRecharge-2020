package frc.robot.logging;

import java.util.Hashtable;
import java.util.HashSet;
import java.util.StringTokenizer;

import frc.robot.Utils;

import java.util.ArrayList;
import java.io.*;

// FILE HAS NOT BEEN CLEANED UP //
public class CSVHelper {

    private String filename;
    private ArrayList<String> topics;
    private ArrayList<String> rowsToAdd;
    private HashSet<String> topicLookup;
    private String lastLine;

    // Precondition: File already has 1 entry that doesn't end in a comma

    public CSVHelper(String filename) throws IOException{
        this.filename = filename;
        // Not using newBufferedReader b/c when instantiated, we want it to error out of the file isn't found
        BufferedReader reader = new BufferedReader(new FileReader(this.filename));
        this.topics    = new ArrayList<String>();
        this.rowsToAdd = new ArrayList<String>();
        try{
            StringTokenizer firstRow = new StringTokenizer(reader.readLine(), ",");
        
            while(firstRow.hasMoreTokens()){
                this.topics.add(firstRow.nextToken());
            }
        }catch (Exception e){}
        this.topicLookup = Utils.arrayListToHashset(this.topics);
        reader.close();
        updateLastLine();
    }

    private void fileNotFound(){
        System.out.println("FILE NOT FOUND (CSVHelper)");
    }

    private BufferedReader newBufferedReader(String filename){
        try {
            return new BufferedReader(new FileReader(filename));
        } catch (IOException E) {
            fileNotFound();
            return null;
        }
    }

    // Check With Bowen -> should the append be false in the cases when there is currently no bool in the constructor?
    private PrintWriter newPrintWriter(String filename, boolean append){
        try {
            return new PrintWriter(new BufferedWriter(new FileWriter(filename, append)));
        } catch (IOException E) {
            fileNotFound();
            return null;
        }
    }

    private BufferedReader newBufferedReader(File file){
        try {
            return new BufferedReader(new FileReader(file));
        } catch (Exception E) {
            fileNotFound();
            return null;
        }
    }

    // Check With Bowen -> should the append be false in the cases when there is currently no bool in the constructor?
    private PrintWriter newPrintWriter(File file){
        try {
            return new PrintWriter(new BufferedWriter(new FileWriter(file)));
        } catch (IOException E) {
            fileNotFound();
            return null;
        }
    }

    private String readLine(BufferedReader reader){
        try {
            return reader.readLine();
        } catch (IOException E){
            fileNotFound();
            return null;
        }
    }

    private void closeReader(BufferedReader reader){
        try {
            reader.close();
        } catch (IOException E){
            fileNotFound();
        }
    }

    private void updateLastLine(){
        BufferedReader reader = newBufferedReader(filename);
        String tempLastLine = "";
        String current = "";
        while((current = readLine(reader)) != null){
            tempLastLine = current;
        }
        this.lastLine = tempLastLine;
        closeReader(reader);
    }

    public Hashtable<String, String> getLastRow(){
        updateLastLine();
        StringTokenizer lastLineString = new StringTokenizer(lastLine, ",");
        Hashtable<String, String> lastrow = new Hashtable<String, String>();
        for(String topic : this.topics){
            String value = "";
            try{
                value = lastLineString.nextToken();
            }catch (Exception e){}
            lastrow.put(topic, value);
        }

        return lastrow;
    }

    public void addRow(Hashtable<String,String> row){
        // Should add a row to the next empty row of the sheet
        // The key of the hashtable should be the name of the topic
        String content = "";
        for(String topic : this.topics){
            String value = row.get(topic);
            if (value == null){
                value = "";
            }
            content += (value + ",");
        }
        this.rowsToAdd.add(content);
    }

    public void writeRows(){
        PrintWriter writer = newPrintWriter(filename, true);
        for (String content : this.rowsToAdd){
            content = content.substring(0, content.length() - 1);
            writer.println(content);
        }
        writer.close();
        this.rowsToAdd = new ArrayList<String>();
    }

    public void addTopic(String topic){
        // Adds a rightmost topic with every cell empty except the top one w/ the topic name
        // Should only work if the topic has not been used before
        if (topicExists(topic)){return;}

        int insertionSpot = filename.length() - ".csv".length(); // Write before the ".csv"
        String newfilename = filename.substring(0, insertionSpot) + "-THIS-IS-A-TEMPORARY-FILE" + filename.substring(insertionSpot);
        File oldFile = new File(filename);
        File newFile = new File(newfilename);

        try {
            newFile.createNewFile();
        } catch (Exception e) {
            System.out.println("ERROR (CSVHelper), couldn't create file: ");
            System.out.println(newfilename);
        };

        BufferedReader reader = newBufferedReader(oldFile);
        PrintWriter    writer = newPrintWriter(newFile);
        
        boolean firstRow = true;
        String line = "";
        while((line = readLine(reader)) != null){
            if(firstRow){
                writer.println(line+","+topic);
                firstRow = false;
            }else{
                writer.println(line);
            }
        }
        if (firstRow){
            // IE: the file is empty
            writer.println(topic);
        }
        oldFile.delete();
        newFile.renameTo(oldFile);
        closeReader(reader);
        writer.close();
        this.topics.add(topic);
        this.topicLookup.add(topic);
    }

    public ArrayList<String> getTopics(){return topics;}

    public boolean topicExists(String topic){
        return this.topicLookup.contains(topic);
    }

}
