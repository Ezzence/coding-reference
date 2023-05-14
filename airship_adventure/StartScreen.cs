
using UdonSharp;
using UnityEngine;
using UnityEngine.UI;
using VRC.SDKBase;
using VRC.Udon;
// Written for Udon Sharp compiler

// Certain features for enums are missing from U#
public enum Role
{
    Captain = 1,
    Engineer = 2,
    Gunner = 3,
    Desparado = 4
}

/// <summary>
/// The Start/Menu Screen. This allows players to change their options within the game, and handles some global game events
/// </summary>
[UdonBehaviourSyncMode(BehaviourSyncMode.Manual)]
public sealed class StartScreen : UdonSharpBehaviour
{
    [Header("Unity Objects")]
    [SerializeField] GameObject objShip;
    [SerializeField] GameObject objScreenPlay;
    [SerializeField] GameObject objScreenOver;
    [SerializeField] GameObject objPostProcess;
    [SerializeField] Inventory udonInventory;
    [SerializeField] UdonBehaviour udonShip;
    [SerializeField] UdonBehaviour udonCloud;
    [SerializeField] UdonBehaviour udonMountain;
    [SerializeField] UdonBehaviour udonMountainLine;

    [SerializeField] Text textDistance;
    [SerializeField] Text textRole;
    [SerializeField] Toggle toggleRoles;
    [SerializeField] Toggle toggleGraphics;
    Object[] editorObjects;

    [Header("Value Fields")]
    [UdonSynced] public Role[] intRoles = {0,0,0,0,0,0,0,0,0,0,0,0}; // size 12 for max_player_count * 2
    [UdonSynced] public bool bPlay = true;
    [UdonSynced] public bool bRoles = false;
    bool bGraphics = false;

    // Local Player
    VRCPlayerApi _player;

    /// <summary>
    /// This function resets certain data to initial values, used for restarting the game after <c>GameOver</c>
    /// </summary>
    public void GameStart()
    {
        if (!Networking.IsMaster)
        {
            return; // only the host may restart the game
        }
        
        Networking.SetOwner(_player, gameObject);
        bPlay = true;
        RequestSerialization();
        objShip.SetActive(bPlay);
        objScreenPlay.SetActive(bPlay);
        objScreenOver.SetActive(!bPlay);
        udonInventory.ResetResources();
        udonInventory.RefreshInventory();
        udonMountain.SendCustomEvent("Reset");
        udonCloud.SendCustomEvent("SpeedMedium");
        udonCloud.SetProgramVariable("distAdventure", 0);
        udonShip.SetProgramVariable("health", 1.0f);
    }

    public void GameOver()
    {
        Networking.SetOwner(_player, gameObject);
        bPlay = false;
        RequestSerialization();
        objScreenPlay.SetActive(bPlay);
        objScreenOver.SetActive(!bPlay);
        udonCloud.SendCustomEvent("SpeedStop");
        textDistance.text = "Distance: " + ((float)udonCloud.GetProgramVariable("distAdventure")).ToString();
    }

    public void ToggleGraphics()
    {
        bGraphics = toggleGraphics.isOn;
        objPostProcess.SetActive(bGraphics);
    }

    public void ToggleRoles()
    {
        if (!Networking.IsMaster)
        {
            return;
        }
        
        Networking.SetOwner(_player, gameObject);
        bRoles = toggleRoles.isOn;
        RequestSerialization();
        udonInventory.SendCustomEventDelayedSeconds("RefreshInventory", 1.0f, VRC.Udon.Common.Enums.EventTiming.Update);

    }

    public void SelectEngineer()
    {
        if (Networking.IsMaster)
        {
            return; // host is always captain
        }

        Networking.SetOwner(_player, gameObject);
        textRole.text = "ENGINEER";
        if (_player.playerId < intRoles.Length)
        {
            intRoles[_player.playerId] = Role.Engineer;
            RequestSerialization();
            udonInventory.SendCustomNetworkEvent(VRC.Udon.Common.Interfaces.NetworkEventTarget.Owner, "RefreshInventory");
        }
    }

    public void SelectGunner()
    {
        if (Networking.IsMaster)
        {
            return; // host is always captain
        }

        Networking.SetOwner(_player, gameObject);
        textRole.text = "GUNNER";
        if (_player.playerId < intRoles.Length)
        {
            intRoles[_player.playerId] = Role.Gunner;
            RequestSerialization();
            udonInventory.SendCustomNetworkEvent(VRC.Udon.Common.Interfaces.NetworkEventTarget.Owner, "RefreshInventory");
        }
    }

    public void SelectDesparado()
    {
        if (Networking.IsMaster)
        {
            return; // host is always captain
        }

        Networking.SetOwner(_player, gameObject);
        textRole.text = "DESPARADO";
        if (_player.playerId < intRoles.Length)
        {
            intRoles[_player.playerId] = Role.Desparado;
            RequestSerialization();
            udonInventory.SendCustomNetworkEvent(VRC.Udon.Common.Interfaces.NetworkEventTarget.Owner, "RefreshInventory");
        }
    }

    public override void OnDeserialization()
    {
        objScreenPlay.SetActive(bPlay);
        objShip.SetActive(bPlay);
        objScreenOver.SetActive(!bPlay);
        textDistance.text = "Distance: " + ((float)udonCloud.GetProgramVariable("distAdventure")).ToString();
        toggleRoles.isOn = bRoles;
    }

    public override void OnPlayerLeft(VRCPlayerApi player)
    {
        base.OnPlayerLeft(player);
        if (Networking.IsMaster)
        {
            toggleRoles.interactable = true;
        }
    }

    public override void OnPlayerJoined(VRCPlayerApi player)
    {
        base.OnPlayerJoined(player);
        if (!Networking.IsOwner(gameObject))
        {
            return;
        }

        if (player.playerId < intRoles.Length)
        {
            // host becomes captain, other players become engineers
            intRoles[player.playerId] = (player.isMaster ? Role.Captain : Role.Engineer);
        }
        RequestSerialization();
        SendCustomNetworkEvent(VRC.Udon.Common.Interfaces.NetworkEventTarget.Owner, "RefreshInventory");
    }

    private void Start()
    {
        _player = Networking.LocalPlayer;
        editorObjects = new Object[]{objShip, objScreenPlay, objScreenOver, objPostProcess, textDistance, textRole, toggleRoles, toggleGraphics};

        foreach (Object obj in editorObjects)
        {
            if (obj == null)
            {
                Debug.Log("Error: Missing StartScreen Asset Reference in Editor");
            }
        }

        if (Networking.IsOwner(gameObject))
        {
            textRole.text = "CAPTAIN";
            toggleRoles.interactable = true;
        }
        else
        {
            textRole.text = "ENGINEER";
            toggleRoles.interactable = false;
        }

        if (!_player.IsUserInVR())
        {
            toggleGraphics.isOn = true;
            objPostProcess.SetActive(true);
        }
    }
}
