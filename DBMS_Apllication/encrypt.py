from cryptography.fernet import Fernet

message = "hello geeks"

key = Fernet.generate_key()

file_obj = open("secret.txt", "w")
file_obj.write(str(key, "utf-8"))
file_obj.close()

fernet = Fernet(key)

encMessage = fernet.encrypt(message.encode())
file_obj = open("encmsg.txt", "w")
file_obj.write(str(encMessage,"utf-8"))
file_obj.close()
