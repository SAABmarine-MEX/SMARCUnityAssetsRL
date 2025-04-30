using UnityEngine;
using System;
using System.Net.Sockets;
using Google.Protobuf;
using Mex;  // your proto package
using System.Linq;

namespace DefaultNamespace.BlueROV2
{
    public class PythonModelClient : MonoBehaviour 
    {
        TcpClient client;
        NetworkStream stream;

        void Start() {
            client = new TcpClient("127.0.0.1", 5005);
            stream = client.GetStream();
            Debug.Log("[Unity] Connected to Python");
        }

        public float[] QueryResiduals(float[] features) {
            // Build request
            var req = new InputFeatures();
            req.Features.AddRange(features);
            byte[] data = req.ToByteArray();
        
            Debug.Log("[Unity] QueryModel FEATURES: " + string.Join(",", features.Take(6)) + " …");

            // Send length prefix
            byte[] lenB = BitConverter.GetBytes(data.Length);
            if (BitConverter.IsLittleEndian) Array.Reverse(lenB);
            stream.Write(lenB,0,4);
            stream.Write(data,0,data.Length);

            // Read reply length
            byte[] rlenB = new byte[4];
            stream.Read(rlenB,0,4);
            if (BitConverter.IsLittleEndian) Array.Reverse(rlenB);
            int L = BitConverter.ToInt32(rlenB,0);

            // Read reply
            byte[] buf = new byte[L]; int read=0;
            while(read<L) read += stream.Read(buf,read,L-read);

            var resp = Prediction.Parser.ParseFrom(buf);
            return resp.Residuals.ToArray();
        }

        void OnApplicationQuit(){
            stream?.Close(); client?.Close();
        }
    }
}
