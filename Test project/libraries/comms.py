from struct import unpack, pack

def message_encode(message_type:str,content:list)->bytes:
    #Encodees message in format (type, coord1, coord2, block id, block action)
    format = "HffH"
    if message_type == "WhereAreYou":
        message = pack(format,1,0,0,0)#Just a ping
    elif message_type == "IAmHere":
        message = pack(format,2,content[0],content[1],0)#coord1, coord2, Null
    elif message_type == "NewBlock":
        message = pack(format,3,content[0],content[1],0)#coord1, coord2, Null
    elif message_type == "BlockRed":
        message = pack(format,4,0,0,content[0])#Null, Null, ID
    elif message_type == "BlockGreen":
        message = pack(format,5,0,0,content[0])#Null, Null, ID
    elif message_type == "MyBlock":
        message = pack(format,6,0,0,content[0])#Null, Null, ID
    return message

    
def message_decode(message:bytes)->(str,list):
    #Decodes the message sent
    data = unpack("HffH",message)
    if data[0] == 1:
        return ("WhereAreYou", [])
    elif data[0] ==2:
        return ("IAmHere", [data[1],data[2]])
    elif data[0] ==3:
        return ("NewBlock", [data[1],data[2],data[3]]) 
    elif data[0] ==4:
        return ("BlockRed", [data[3]])
    elif data[0] ==5:
        return ("BlockGreen", [data[3]])
    elif data[0] ==6:
        return ("MyBlock", [data[3]])