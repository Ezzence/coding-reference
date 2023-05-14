
using UdonSharp;
using UnityEngine;
using UnityEngine.UI;
using VRC.SDKBase;
using VRC.Udon;
// Written for Udon Sharp compiler

/// <summary>
/// Manages all resources gathered by the crew of the airship, and runs the display panel at the back
/// </summary>
[UdonBehaviourSyncMode(BehaviourSyncMode.Manual)]
public sealed class Inventory : UdonSharpBehaviour
{
    [Header("Unity Objects")]
    [SerializeField]  StartScreen udonStart;
    [SerializeField]  UdonBehaviour udonCannon;
    [SerializeField]  UdonBehaviour udonShip;
    [SerializeField]  UdonBehaviour udonFalcon;
    [SerializeField]  Text textWood;
    [SerializeField]  Text textIron;
    [SerializeField]  Text textParts;
    [SerializeField]  Text textRoles;
    [SerializeField]  Text textCaptain;
    [SerializeField]  Text textEngineers;
    [SerializeField]  Text textGunners;
    [SerializeField]  Text textDesparados;
    [SerializeField]  Text textDifficulty;
    Text[] textObjs;
    [UdonSynced] string strEngineers;
    [UdonSynced] string strGunners;
    [UdonSynced] string strDesparados;

    [Header("Value Fields")]
    [Tooltip("Amount of iron resource the players posess")]
    [UdonSynced] public int countIron = 4;

    [Tooltip("Amount of wood resource the players posess")]
    [UdonSynced] public int countWood = 1;

    [Tooltip("Amount of parts resource the players posess")]
    [UdonSynced] public int countParts = 1;

    [Tooltip("Procedural difficulty level, higher value means more difficulty")]
    [UdonSynced] public int countDifficulty = 1;

    const int countIronInitial = 4;
    const int countWoodInitial = 1;
    const int countPartsInitial = 1;
    const int countDifficultyInitial = 1;

    // Local Player
    VRCPlayerApi _player;

    /// <summary>
    /// The owner (host) of <c>Inventory</c> updates own resource and label values, and then syncs them to other players
    /// </summary>
    public void RefreshInventory()
    {
        if (!Networking.IsOwner(gameObject))
        {
            return; // only the owner can update synced variables
        }

        VRCPlayerApi[] players = new VRCPlayerApi[12];
        VRCPlayerApi.GetPlayers(players);
        Role[] roles = udonStart.intRoles;
        strEngineers = ""; strGunners = ""; strDesparados = "";

        foreach (VRCPlayerApi player in players)
        {
            if (player == null)
            {
                continue;
            }

            if (player.playerId >= roles.Length)
            {
                continue;
            }

            switch (roles[player.playerId])
            {
                case Role.Engineer:
                    strEngineers += player.displayName + ",";
                    break;
                case Role.Gunner:
                    strGunners += player.displayName + ",";
                    break;
                case Role.Desparado:
                    strDesparados += player.displayName + ",";
                    break;
            }
        }
        RequestSerialization();
        RefreshText();    
    }

    /// <summary>
    /// This is an Udon event function, and it cannot have arguments. Used for airship resource modification on host. 
    /// </summary>
    public void AddIronInventory()
    {
        countIron++;
        RefreshInventory();
    }
    public void AddWoodInventory()
    {
        countWood++;
        RefreshInventory();
    }

    public void CostConditionShip()
    {
        countWood -= (int)udonShip.GetProgramVariable("conditionCostW"); // Some of these costs increase with each upgrade
        RefreshInventory();
    }
    public void CostEfficiencyShip()
    {
        countIron -= (int)udonShip.GetProgramVariable("efficiencyCost");
        countWood -= (int)udonShip.GetProgramVariable("efficiencyCostW");
        RefreshInventory();
    }
    public void CostArmor()
    {
        countWood -= (int)udonShip.GetProgramVariable("armorCostW");
        RefreshInventory();
    }
    public void CostCondition()
    {
        countIron -= (int)udonCannon.GetProgramVariable("conditionCost");
        countWood -= (int)udonCannon.GetProgramVariable("conditionCostW");
        RefreshInventory();
    }
    public void CostEfficiency()
    {
        countIron -= (int)udonCannon.GetProgramVariable("efficiencyCost");
        RefreshInventory();
    }
    public void CostRange()
    {
        countIron -= (int)udonCannon.GetProgramVariable("rangeCost");
        RefreshInventory();
    }
    public void CostReload()
    {
        countIron -= (int)udonCannon.GetProgramVariable("reloadCost");
        RefreshInventory();
    }
    public void CostBuildFalcon()
    {
        countIron -= (int)udonFalcon.GetProgramVariable("buildCost");
        countWood -= (int)udonFalcon.GetProgramVariable("buildCostW");
        RefreshInventory();
    }
    public void CostRangeFalcon()
    {
        countIron -= (int)udonFalcon.GetProgramVariable("rangeCost");
        RefreshInventory();
    }

    public void ResetResources()
    {
        countIron = countIronInitial;
        countWood = countWoodInitial;
        countParts = countPartsInitial;
        countDifficulty = countDifficultyInitial;
    }

    /// <summary>
    /// Udon behaviour function called after synchornizing GameObjects between players
    /// </summary>
    public override void OnDeserialization()
    {
        RefreshText();
    }

    public override void OnOwnershipTransferred(VRCPlayerApi player)
    {
        // base.OnOwnershipTransferred();
        if (Networking.IsOwner(gameObject))
        {
            RefreshInventory();
        }
    }

    private void RefreshText()
    {
        // Try-Catch blocks are not supported by Udon#
        foreach (Text t in textObjs)
        {
            if (t == null)
            {
                Debug.Log("Error: Missing Inventory Asset Reference in Editor");
            }
        }

        textIron.text = countIron.ToString();
        textWood.text = countWood.ToString();
        textParts.text = countParts.ToString();
        textDifficulty.text = countDifficulty.ToString();
        textCaptain.text = _player.displayName;
        textEngineers.text = strEngineers;
        textGunners.text = strGunners;
        textDesparados.text = strDesparados;
        textRoles.text = udonStart.bRoles.ToString();
    }

    private void Start()
    {
        _player = Networking.LocalPlayer;
        textObjs = new Text[9] {textWood, textIron, textParts, textRoles, textCaptain, textEngineers, textGunners, textDesparados, textDifficulty};

        if (Networking.IsOwner(gameObject))
        {
            ResetResources();
            RefreshInventory();
        }
        else
        {
            SendCustomNetworkEvent(VRC.Udon.Common.Interfaces.NetworkEventTarget.Owner, "RefreshInventory");
        }
    }
}
