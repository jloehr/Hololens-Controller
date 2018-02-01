using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BoxMover : MonoBehaviour {

    public GameObject BoxCollections;
    public GameObject BoxPrefab;

    private Transform SelectedBox;

    public void Tap()
    {
        if (SelectedBox != null)
        {
            SelectedBox.gameObject.layer = LayerMask.NameToLayer("Box");
            SelectedBox = null;
            return;
        }

        RaycastHit[] Hits = Physics.RaycastAll(transform.position, transform.forward);

        foreach (RaycastHit Hit in Hits)
        {
            if(Hit.transform.gameObject.layer == LayerMask.NameToLayer("Box"))
            {
                SelectedBox = Hit.transform;
                SelectedBox.gameObject.layer = LayerMask.NameToLayer("Ignore Raycast");
                return;
            }
        }

        if(Hits.Length > 0)
        {
            GameObject New = Instantiate(BoxPrefab, Hits[0].point, Quaternion.Euler(Hits[0].normal), BoxCollections.transform);
            return;
        }
    }

	void Update ()
    {
        if (SelectedBox != null)
        {
            RaycastHit Hit;
            if(Physics.Raycast(transform.position, transform.forward, out Hit, Mathf.Infinity))
            {
                SelectedBox.transform.position = Hit.point;
                SelectedBox.transform.rotation = Quaternion.Euler(Hit.normal);
            }
        }
	}

    void OnEnable()
    {
        BoxCollections.SetActive(true);
    }

    void OnDisable()
    {
        BoxCollections.SetActive(false);
    }
}
