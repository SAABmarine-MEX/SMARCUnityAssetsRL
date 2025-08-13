using UnityEngine;

namespace BlueROV2.Physics
{
    public class Tether : MonoBehaviour
    {
        private bool useTetherDynamics = true;
        private bool useTetherVisuals = true;
        
        [Header("ROV & Cable Properties")]
        public ArticulationBody rovAb;
        
        // https://bluerobotics.com/store/cables-connectors/cables/fathom-rov-tether-rov-ready/
        private float rhoMat = 1000f;            // cable material density (kg/m³)
        private float diameter = 0.0076f;         // m
        private float rhoWater = 1025f;         // kg/m³
        private float dragCd = 1.5f; //1.2f           // cable drag coefficient. // https://link.springer.com/chapter/10.1007/978-94-009-4207-3_5 says 2.0. general cylinder is 1.2
        
        [Header("Attachment Offset (body frame)")]
        // X forward, Y right, Z up
        private Vector3 attachOffset = new Vector3(-0.16f, 0f, -0.22f); // https://bluerobotics.com/store/rov/bluerov2-accessories/brov-payload-skid/

        private float bPerMeter;               // net buoyancy per meter (N/m)

        private float g = 9.82f;
        
        [Header("Line Renderer Visuals")]
        public float surfaceZ = 0f;
        public Material lineMaterial;
        private LineRenderer line;


        void Start() {
            rovAb = GetComponentInChildren<ArticulationBody>();
            if (rovAb == null)
                Debug.LogError("ArticulationBody component not found on this GameObject.");

            
            float A = Mathf.PI * diameter * diameter / 4f;
            bPerMeter = (rhoWater - rhoMat) * A * g;
            
            
            // Create LineRenderer
            line = gameObject.GetComponent<LineRenderer>();
            if (line == null)
            {
                line = gameObject.AddComponent<LineRenderer>();
            }

            line.positionCount = 2;
            line.startWidth = diameter;
            line.endWidth = diameter;

            // Set default material if not assigned
            if (lineMaterial != null)
            {
                line.material = lineMaterial;
            }
            else
            {
                line.material = new Material(Shader.Find("Sprites/Default")); // fallback shader
                line.material.color = Color.yellow;
            }

        }

        public void SetUseTetherDynamics(bool value)
        {
            useTetherDynamics = value;
        }
        public void SetUseTetherVisuals(bool value)
        {
            useTetherVisuals = value;
        }

        void FixedUpdate()
        {
            //if (!useTetherDynamics)
            //{
            //    line.positionCount = 0;
            //    return;
            //}
            
            // Compute world‐space attach point
            Vector3 worldAttach = rovAb.transform.TransformPoint(attachOffset);
            
            if (useTetherDynamics)
            {
                // 1) Depth (positive down)
                float h = Mathf.Max(0f, -rovAb.transform.position.y); 
                
                /*
                // 2) Vertical buoyant force
                //Vector3 Fbuoy = bPerMeter * h * Vector3.up;
                // 3) Horizontal drag on cable
                Vector3 v = rovAb.linearVelocity; // world velocity of ROV
                Vector3 vHoriz = new Vector3(v.x, 0f, v.z); // horizontal only
                float L = h;
                Vector3 Fdrag = -0.5f * rhoWater * dragCd * diameter * L
                                * vHoriz.magnitude * vHoriz;
                 */
            
                
                // 2) Forces
                Vector3 Fbuoy = bPerMeter * h * Vector3.up;
                Vector3 v = rovAb.linearVelocity;
                Vector3 vHoriz = new Vector3(v.x, 0f, v.z);
                float L = h;
                Vector3 Fdrag = -0.5f * rhoWater * dragCd * diameter * L * vHoriz.magnitude * vHoriz;
                //Debug.Log(Fdrag);
                
                // Total tether force
                Vector3 Ftether = Fbuoy + Fdrag;
                // Playing around with z-component (x-component in NED) since from comparing to the real mocap data,
                // it seems to not contribute that much. Lowered it and the sim seemed to match better with real
                Ftether = new Vector3(Ftether.x, Ftether.y, Ftether.z*0.01f); 
                
                // Apply force at that point (also generates moment)
                rovAb.AddForceAtPosition(Ftether, worldAttach);
            }

            if (useTetherVisuals)
            {
                // Tether visualization
                //Vector3 worldAttach = rovAb.transform.position + transform.TransformDirection(attachOffset);
                // 3) Attachment point in world space
                Vector3 worldSurface = new Vector3(worldAttach.x, surfaceZ, worldAttach.z);

                line.SetPosition(0, worldAttach);
                //Debug.Log(worldAttach);
                line.SetPosition(1, worldSurface);
                
            }
            else
            {
                line.positionCount = 0;
            }
            
            
            
            
            
        }
    }
}