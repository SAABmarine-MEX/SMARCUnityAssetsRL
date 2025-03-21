using UnityEngine;

public class CheckpointSingle : MonoBehaviour
{
    public int checkpointIndex; // TODO: now this is index is set from unity. make it instead so in trackcheckpoints or somewhere else gives the index by loop over Checkpoints in Unity
    /*
    private TrackCheckpoints trackCheckpoints;
    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag == "Player")
        {
            Debug.Log("CHEEEEEEEECK");
            //trackCheckpoints.PlayerThroughCheckpoint(this);
        }
    }

    public void SetTrackCheckpoints(TrackCheckpoints trackCheckpoints)
    {
        this.trackCheckpoints = trackCheckpoints;
    }
    */
}
