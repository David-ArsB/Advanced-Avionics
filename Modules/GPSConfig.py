def calculateChecksum (msg):
    checksum = 0;
    for (i in msg && i < 32; i++)
        checksum ^= msg[i]

    return checksum


INLINE int nemaMsgSend (const char *msg)
{
    char checksum[8];
    snprintf(checksum, sizeof(checksum)-1, F("*%.2X"), calculateChecksum(msg));
    module.print("$");
    module.print(msg);
    module.println(checksum);
}

inline int nemaMsgDisable (const char *nema)
{
    if (strlen(nema) != 3) return 0;

    char tmp[32];
    snprintf(tmp, sizeof(tmp)-1, F("PUBX,40,%s,0,0,0,0"), nema);
    //snprintf(tmp, sizeof(tmp)-1, F("PUBX,40,%s,0,0,0,0,0,0"), nema);
    nemaMsgSend(tmp);

    return 1;
}

inline int nemaMsgEnable (const char *nema)
{
    if (strlen(nema) != 3) return 0;

    char tmp[32];
    snprintf(tmp, sizeof(tmp)-1, F("PUBX,40,%s,0,1,0,0"), nema);
    //snprintf(tmp, sizeof(tmp)-1, F("PUBX,40,%s,0,1,0,0,0,0"), nema);
    nemaMsgSend(tmp);

    return 1;
}