using UnityEngine;
using System;
using System.Collections;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Globalization;

public class Spheres : MonoBehaviour
{
    // 1. Declare Variables
    Thread receiveThread; //1
    UdpClient client; //2
    int port; //3
    float[] coord = new float[8];
    public GameObject sphere; //4
    bool updateCoord = false;
    // 2. Initialize variables
    // Start is called before the first frame update
    void Start()
    {
        port = 5065; //1 
        sphere = GameObject.Find("Sphere");
        InitUDP(); //4

    }

    // 3. InitUDP
    private void InitUDP()
    {
        print("UDP Initialized");

        receiveThread = new Thread(new ThreadStart(ReceiveData)); //1 
        receiveThread.IsBackground = true; //2
        receiveThread.Start(); //3

    }
    // 4. Receive Data
    private void ReceiveData()
    {
        client = new UdpClient(port); //1
        while (true) //2
        {
            try
            {
                IPEndPoint anyIP = new IPEndPoint(IPAddress.Parse("0.0.0.0"), port); //3
                byte[] data = client.Receive(ref anyIP); //4
                print("Received data:");
                print(data);

                string text = Encoding.UTF8.GetString(data); //5
                print("Encoding UTF8 text:");
                print(text);
                string[] strArray = text.Split(',');
                print("Splitted array:");
                print(strArray[1]);
                float[] coord1 = new float[8];
                for (int i = 0; i < strArray.Length; ++i)
                {
                    float x = float.Parse(strArray[i], CultureInfo.InvariantCulture.NumberFormat);
                    coord1[i] = x;
                    print("Coord" + Convert.ToString(i) + Convert.ToString(coord1[i]));
                    // print("<<"+x);
                }
                //print(coord1);
                if (updateCoord) 
                {
                    coord = coord1;
                    updateCoord = false;
                }



            }
            catch (Exception e)
            {
                print(e.ToString()); //7
            }
        }
    }


    // Update is called once per frame
    void Update()
    {
        if (!updateCoord)
        {
            for (int i = 1; i < 4; i++)
            {
                string obj = "Sphere (" + Convert.ToString(i) + ")";
                GameObject spherei = sphere.transform.Find(obj).gameObject;
                if (coord[0] == i)
                {
                    spherei.transform.rotation = new Quaternion(coord[1], coord[2], coord[3], coord[4]);
                    spherei.transform.position = new Vector3(coord[5]*15, coord[6]*5, coord[7]*5);
                }
                // print(obj);
                // print(coord[2*i]/10);
                // print(coord[2*i+1]/10

                updateCoord = true;
            }
        }
    }
}