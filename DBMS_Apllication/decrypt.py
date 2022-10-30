from cryptography.fernet import Fernet


read_file = open("secret.txt", "r")
key = bytes(read_file.read(), "utf-8")
read_file.close()

read_file = open("encmsg.txt", "r")
enc_msg = bytes(read_file.read(), "utf-8")
read_file.close()

fernet = Fernet(key)
decMessage = fernet.decrypt(enc_msg).decode()
print(decMessage)




