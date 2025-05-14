using System;
using System.Linq;
using System.Net.Http;
using System.Threading.Tasks;
using Google.Protobuf;
using Mex; // your generated proto namespace
using UnityEngine;

public class PythonModelHttpClient : MonoBehaviour
{
    // reuse a single HttpClient instance per AppDomain
    private static readonly HttpClient client = new HttpClient
    {
        BaseAddress = new Uri("http://127.0.0.1:8000/predict")
    };
    /*
    async void Start()
    {
        // example usage
        
        float[] features = new float[] { 0.1f, 0.2f, };
        try
        {
            var residuals = await QueryResidualsAsync(features);
            Debug.Log("Got residuals: " + string.Join(",", residuals));
        }
        catch (Exception ex)
        {
            Debug.LogError("Error querying model: " + ex);
        }
    }
    */
    
    public async Task<float[]> QueryResidualsAsync(float[] features)
    {
        // 1) serialize your request proto
        var req = new InputFeatures();
        req.Features.AddRange(features);
        byte[] payload = req.ToByteArray();
        
        // 2) wrap in HttpContent
        using var content = new ByteArrayContent(payload);
        content.Headers.ContentType =
            new System.Net.Http.Headers.MediaTypeHeaderValue("application/octet-stream");
        
        // 3) POST to your FastAPI endpoint (e.g. /predict)
        HttpResponseMessage response = await client.PostAsync("predict", content);
        response.EnsureSuccessStatusCode();
        
        // 4) read the raw response bytes
        byte[] respBytes = await response.Content.ReadAsByteArrayAsync();
        
        // 5) parse the protobuf reply
        var reply = Prediction.Parser.ParseFrom(respBytes);
        return reply.Residuals.ToArray();
    }

    
    /*
    async void FixedUpdate()
    {
        float[] features = new float[18];
        try
        {
            var residuals = await QueryResidualsAsync(features);
            Debug.Log("Got residuals: " + string.Join(",", residuals));
        }
        catch (Exception ex)
        {
            Debug.LogError("Error querying model: " + ex);
        }
    }
    */
    

    void OnApplicationQuit()
    {
        client.Dispose();
    }
}