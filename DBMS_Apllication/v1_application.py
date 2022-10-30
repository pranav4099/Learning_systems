import pandas as pd
from tabulate import tabulate
import os
import time
import sqlite3


class dbms_application:
    def __init__(self):
        self.connection_obj = sqlite3.connect('database.db')
        self.cursor_obj = self.connection_obj.cursor()
        self.starting_port = 909
        self.test()

    def print_menu(self):
        os.system('clear')
        self.empty_line(1)
        print("Which type of user: ")
        print("1. Retail buyer")
        print("2. Wholesale buyer")
        print("3. Port_authority")
        print("4. Mover")
        print("5. Manager")
        print("6. Seller")
        print("7. Insurance_company")
        print("8. ----- EXIT.")
        self.empty_line(1)

    def accept_input(self, lower, higher, sentence):
        value = None
        while(True):
            try:
                value = int(input(str(sentence)))
                if(lower<value<higher):
                    break
            except:
                pass
        return value
            

    def request_login(self):
        while(True):
            self.print_menu()
            main_menu_option = self.accept_input(0,9,"Enter the option: ")
            if(main_menu_option==8):
                break
            user_id = self.accept_input(-1,1000000,"Enter the User_ID: ")
            
            if(main_menu_option==1 or main_menu_option==2):
                retail_buyer_statement = "select * from retailer where buyer_id="+str(user_id)
                wholesaler_buyer_statement = "select * from wholesaler where buyer_id="+str(user_id)
                retailer_df, table_info_retailer=self.get_data(retail_buyer_statement,retrun_pd_info=True, show_table=False)
                wholesaler_df, table_info_wholesaler=self.get_data(wholesaler_buyer_statement,retrun_pd_info=True, show_table=False)

                if(table_info_retailer[0]==0 and table_info_wholesaler[0]==0):
                    print("Username does not exist.")
                    time.sleep(2)
                else:
                    buyer_type = "retailer" if(table_info_retailer[0]) else "wholesaler"
                    user_entered_password = str(input("Enter the password: "))
                    if(buyer_type=="retailer"):
                        password = retailer_df['password'].values[0]
                    else:
                        password = wholesaler_df['password'].values[0]
                    if(user_entered_password!=password):
                        print("Wrong password")
                        time.sleep(2)
                        continue

                    self.buyer(user_id, buyer_type)

            elif(main_menu_option==3):
                statement = "select * from port_authority where port_authority_id="+str(user_id)
                df, table_info=self.get_data(statement,retrun_pd_info=True, show_table=False)
                if(table_info[0]==0):
                    print("Username does not exist.")
                    time.sleep(2)
                else:
                    user_entered_password = str(input("Enter the password: "))
                    password = df['password'].values[0]
                    if(user_entered_password!=password):
                        print("Wrong password")
                        time.sleep(2)
                        continue

                    self.port_authority_actions(user_id)

            elif(main_menu_option==4):
                mover_select_statement = "select * from mover where worker_id="+str(user_id)
                df, table_info=self.get_data(mover_select_statement, retrun_pd_info=True, show_table=False)
                if(table_info[0]==0):
                    print("Username does not exist.")
                    time.sleep(2)
                else:
                    user_entered_password = str(input("Enter the password: "))
                    password = df['password'].values[0]
                    if(user_entered_password!=password):
                        print("Wrong password")
                        time.sleep(2)
                        continue

                    self.worker_mover(user_id)


            elif(main_menu_option==5):
                manager_select_statement = "select * from manager where worker_id="+str(user_id)
                df, table_info=self.get_data(manager_select_statement, retrun_pd_info=True, show_table=False)
                if(table_info[0]==0):
                    print("Username does not exist.")
                    time.sleep(2)
                else:
                    user_entered_password = str(input("Enter the password: "))
                    password = df['password'].values[0]
                    if(user_entered_password!=password):
                        print("Wrong password")
                        time.sleep(2)
                        continue

                    self.worker_manager(user_id)
                
            if(main_menu_option==6):
                statement = "select * from manufacturer where seller_id="+str(user_id)
                df, table_info=self.get_data(statement,retrun_pd_info=True, show_table=False)

                if(table_info[0]==0):
                    print("Username does not exist.")
                    time.sleep(2)
                else:
                    user_entered_password = str(input("Enter the password: "))
                    password = df['password'].values[0]
                    if(user_entered_password!=password):
                        print("Wrong password")
                        time.sleep(2)
                        continue

                    self.seller(user_id)


            if(main_menu_option==7):
                statement = "select * from insurance_company where seller_id="+str(user_id)
                df, table_info=self.get_data(statement,retrun_pd_info=True, show_table=False)

                if(table_info[0]==0):
                    print("Username does not exist.")
                    time.sleep(2)
                else:
                    user_entered_password = str(input("Enter the password: "))
                    password = df['password'].values[0]
                    if(user_entered_password!=password):
                        print("Wrong password")
                        time.sleep(2)
                        continue
                    self.insurance_company(user_id)
            

    def show_df(self,df):
        os.system('clear')
        pd.set_option('display.width', len(df.columns))
        self.empty_line(2)
        print(tabulate(df, headers='keys', tablefmt='psql'))
        self.empty_line(2)

    def empty_line(self, value):
        i=0
        while(i<value):
            print("\n")
            i=i+1

    def port_authority_actions(self, user_id):
        os.system("clear")
        port_select_statement = "select * from port_authority where port_authority_id="+str(user_id)
        df, table_info=self.get_data(port_select_statement, retrun_pd_info=True, show_table=False)
        assigned_port = df['port_id'].values[0]
        while(True):
            os.system("clear")
            print("1. Update safety")
            print("2. Log out")
            main_menu_option = self.accept_input(0,3,"Enter the option: ")
            if(main_menu_option==1):
                statement = '''select * from cargo where port_authority_id='''+str(user_id)
                while(True):
                    df, table_info=self.get_data(statement,retrun_pd_info=True)
                    option = self.accept_input(-2,table_info[0],"Enter the idx of the item, or -1 to return to main menu: ")
                    if(option==-1):
                        break
                    cargo_id = df['cargo_id'].values[option]
                    seller_id = df['seller_id'].values[option]
                    safety_res = self.accept_input(-1,2,"Enter 0 for False, 1 for True: ")
                    safety_res = True if(safety_res) else False
                    update_safety_statement = "update cargo set safety_check="+str(safety_res)+" where cargo_id="+str(cargo_id)+" and seller_id="+str(seller_id)
                    warehouse_id_update = "update cargo set warehouse_id=1 where cargo_id="+str(cargo_id)+" and seller_id="+str(seller_id)
                    add_to_warehouse_statement = "insert into holds values(1,"+str(seller_id)+","+str(cargo_id)+")"
                    self.cursor_obj.execute(update_safety_statement)
                    self.cursor_obj.execute(warehouse_id_update)
                    self.cursor_obj.execute(add_to_warehouse_statement)
                    print("Status updated succesfully")
                    time.sleep(1)
            if(main_menu_option==2):
                break

                
    def seller(self,seller_id):
        while(True):
            os.system("clear")
            print("1. View orders")
            print("2. view catalog")
            print("3. log out")
            main_menu_option = self.accept_input(0,4,"Enter the option: ")
            if(main_menu_option==1):
                select_statement = "select * from cargo where seller_id="+str(seller_id)
                df, table_info=self.get_data(select_statement, retrun_pd_info=True)
                _ = input("Press any key to exit")
                continue
            elif(main_menu_option==2):
                select_statement = "select * from catalog where seller_id="+str(seller_id)
                df, table_info=self.get_data(select_statement, retrun_pd_info=True)
                option=0
                while(True):
                    df, table_info=self.get_data(select_statement, retrun_pd_info=True)
                    option = self.accept_input(0,5,"Enter 1 to update, 2 to delete, 3 to add new item, 4 to exit: ")
                    if(table_info[0]==0 and option!=3):
                        print("No entries detected, please insert values.")
                        time.sleep(1)
                        continue
                    if(option==1):
                        item_idx = self.accept_input(-1,int(table_info[0]),"Enter the IDX of the item to be updated: ")
                        catalog_id = df['catalog_id'].values[item_idx]
                        # seller_id = df['seller_id'].values[item_idx]
                        updated_price = self.accept_input(0,100000000,"Enter the updated price of the item: ")
                        update_statement = "update catalog set item_price="+str(updated_price)+" where catalog_id="+str(catalog_id)+" and seller_id="+str(seller_id)
                        self.cursor_obj.execute(update_statement)
                        print("Update succesfull.")
                        time.sleep(1)
                        continue
                    if(option==2):
                        item_idx = self.accept_input(-1,int(table_info[0]),"Enter the IDX of the item to be deleted: ")
                        catalog_id = df['catalog_id'].values[item_idx]
                        # seller_id = df['seller_id'].values[item_idx]
                        delete_statement = "delete from catalog where catalog_id="+str(catalog_id)+" and seller_id="+str(seller_id)
                        self.cursor_obj.execute(delete_statement)
                        continue
                    if(option==3):
                        if(table_info[0]==0):
                            valid_catalog_id=1
                        else:
                            latest_catalog_id = int(df['catalog_id'].values[table_info[0]-1])
                            valid_catalog_id = latest_catalog_id+1
                        item_name = str(input("Enter the name of the item to be added: "))
                        item_price = self.accept_input(0,100000000,"Enter the price of the item to be added: ")
                        insert_statement = "INSERT INTO catalog VALUES (" + str(valid_catalog_id)+","+str(seller_id)+",'"+str(item_name)+"',"+str(item_price)+")"
                        self.cursor_obj.execute(insert_statement)
                        continue
                    if(option==4):
                        break 
            elif(main_menu_option==3):
                break
        


    def buyer(self,buyer_id, type):
        purchase_type = "purchase_retailer" if(type=="retailer") else "purchase_wholesaler"
        while(True):
            os.system("clear")
            print("1. View orders")
            print("2. view catalog")
            print("3. Buy/Update Insurance")
            print("4. log out")
            main_menu_option = self.accept_input(0,5,"Enter the option: ")
            
            if(main_menu_option==1):
                select_statement = "select * from "+str(purchase_type)+" where buyer_id="+str(buyer_id)
                df, table_info=self.get_data(select_statement, retrun_pd_info=True)
                _ = input("Press any key to exit")
                continue
            
            if(main_menu_option==2):
                select_statement = "select * from catalog"
                while(True):
                    df, table_info=self.get_data(select_statement, retrun_pd_info=True)
                    option = self.accept_input(-1,2,"Select 1 to buy, 0 to return to main menu: ")
                    if(option==1):
                        item_idx = self.accept_input(-1,int(table_info[0]),"Enter the IDX of the item you want to purchase: ")
                        catalog_id = df['catalog_id'].values[item_idx]
                        seller_id = df['seller_id'].values[item_idx]
                        quantity_upper_limit = 101 if(purchase_type=="purchase_retailer") else 1000000
                        quantity = self.accept_input(0, quantity_upper_limit, "Enter the quantity of purchase. For retailers, maximum is 100: ")
                        destination = str(input("Enter the destination city: "))
                        sekect_cargo_for_seller_id = "select * from cargo where seller_id="+str(seller_id)
                        df, table_info=self.get_data(sekect_cargo_for_seller_id, retrun_pd_info=True)
                        if(table_info[0]==0):
                            valid_cargo_id = 1
                        else:
                            valid_cargo_id = df['cargo_id'].values[-1] + 1                        
                        make_cargo_statement = "insert into cargo values("+str(valid_cargo_id)+","+str(seller_id)+",'"+str(destination)+"',NULL,"+str(1)+","+str(1)+ ",NULL)"
                        make_purchase_statement = "insert into "+str(purchase_type)+" values("+str(buyer_id)+","+str(catalog_id)+","+str(seller_id)+","+str(valid_cargo_id)+","+str(quantity)+ ",NULL)"
                        self.cursor_obj.execute(make_cargo_statement)
                        self.cursor_obj.execute(make_purchase_statement)
                        print("Purchase succesfull.")
                        time.sleep(2)
                    if(option==0):
                        break
            
            if(main_menu_option==3):
                insurace_select_statement = "select company_id Insurance_company from insurance_purchase where buyer_id="+str(buyer_id)
                while(True):
                    df, table_info=self.get_data(insurace_select_statement, retrun_pd_info=True)
                    option = self.accept_input(-1,3,"Select 1 to buy, 2 to drop insurance, 0 to return to main menu: ")
                    if(option==1):
                        if(table_info[0]>0):
                            print("Insurance already purchased. Please drop it to purchase anathor.")
                            time.sleep(2)
                            continue
                        insurance_company_select_statement = "select company_id Company ,listing_price Price from insurance_company"
                        df, table_info=self.get_data(insurance_company_select_statement, retrun_pd_info=True)
                        company = self.accept_input(-1,table_info[0],"Select IDX of the insurance company: ")
                        company_id = df['Company'].values[company]
                        insurance_purchase_statement = "insert into insurance_purchase values("+str(company_id)+","+str(buyer_id)+")"
                        self.cursor_obj.execute(insurance_purchase_statement)
                        print("insurance purchased.")
                        time.sleep(2)
                    
                    elif(option==2):
                        if(table_info[0]==0):
                            print("Please purchase insurance first.")
                            time.sleep(2)
                            continue
                        company_id = df['Insurance_company'].values[0]
                        delete_insurance_statement = "delete from insurance_purchase where company_id="+str(company_id)+" and buyer_id="+str(buyer_id)
                        self.cursor_obj.execute(delete_insurance_statement)

                    else:
                        break
                
            if(main_menu_option==4):
                            break


    def insurance_company(self, company_id):
        company_select_statemnt = "select listing_price from insurance_company where company_id="+str(company_id)
        while(True):
            df, table_info=self.get_data(company_select_statemnt, retrun_pd_info=True)
            print("1. Update listing price")
            print("2. View buyers")
            print("3. Log out")
            main_menu_option = self.accept_input(0,4,"Enter the option: ")
            if(main_menu_option==1):
                company_select_statemnt = "select listing_price from insurance_company where company_id="+str(company_id)
                df, table_info=self.get_data(company_select_statemnt, retrun_pd_info=True)
                updated_price = self.accept_input(0,10000000,"Enter updated price: ")
                price_update_statement = "update insurance_company set listing_price="+str(updated_price)+" where company_id="+str(company_id)
                self.cursor_obj.execute(price_update_statement)
                print("Price updated..")
                time.sleep(1)
                continue
            elif(main_menu_option==2):
                select_buyers_statement = "select buyer_id from insurance_purchase where company_id="+str(company_id)
                df, table_info=self.get_data(select_buyers_statement, retrun_pd_info=True)
                _ = input("Enter any key to return to main menu")
                continue
            else:
                break


    def worker_mover(self, worker_id):
        while(True):
            os.system("clear")
            print("1. Move cargo")
            print("2. log out")
            main_menu_option = self.accept_input(0,3,"Enter the option: ")
        
            if(main_menu_option==1):
                warehouse_select_statement = "select warehouse_id from mover where worker_id="+str(worker_id)
                df, table_info=self.get_data(warehouse_select_statement, retrun_pd_info=True, show_table=False)
                assigned_warehouse = df['warehouse_id'].values[0]
                if(assigned_warehouse==None):
                    print("Contact manager, to be assigned to a warehouse.")
                    time.sleep(2)
                    break
                while(True):
                    worker_select_statement = "select c.cargo_id, c.seller_id, c.destination, c.warehouse_id from cargo c join holds h where c.cargo_id=h.cargo_id and c.seller_id=h.seller_id and h.warehouse_id="+str(assigned_warehouse)
                    df, table_info=self.get_data(worker_select_statement, retrun_pd_info=True)
                    option = self.accept_input(-1,2,"Enter 1 to move cargo, 0 to return to main menu: ")
                    if(option==1):
                        if(table_info[0]==0):
                            print("No cargo to move.")
                            time.sleep(2)
                            continue
                        index = self.accept_input(-1,table_info[0],"Enter the IDX of the cargo to moved to the next warehouse: ")
                        cargo_id = df['cargo_id'].values[index]
                        seller_id = df['seller_id'].values[index]
                        if(assigned_warehouse==1):
                            update_statement = "update holds set warehouse_id=2 "+"where cargo_id="+str(cargo_id)+" and seller_id="+str(seller_id)
                            print("Cargo moved to warehouse 2")
                            time.sleep(1)
                        if(assigned_warehouse==2):
                            update_statement = "delete holds where cargo_id="+str(cargo_id)+" and seller_id="+str(seller_id)
                            print("Cargo moved to destination.")
                            time.sleep(1)
                        self.cursor_obj.execute(update_statement)
                    if(option==0):
                        break

            if(main_menu_option==2):
                break


    def worker_manager(self, user_id):
        while(True):
            print("1. Change mover workplace")
            print("2. Log out")
            main_menu_option = self.accept_input(0,3,"Enter the option: ")
            if(main_menu_option==1):
                while(True):
                    worker_select_statement = "select worker_id, warehouse_id from mover where manager_id="+str(user_id)
                    df, table_info=self.get_data(worker_select_statement, retrun_pd_info=True)
                    if(table_info[0]==0):
                        print("You do not manage any mover. Contact DBA to update status.")
                        break
                    mover_option = self.accept_input(-2,table_info[0],"Enter the IDX, or -1 to exit: ")
                    if(mover_option==-1):
                        break
                    mover_id = df['mover_id'].values[mover_option]
                    warehouse_select_statement = "select warehouse_id from warehouse"
                    df, table_info=self.get_data(warehouse_select_statement, retrun_pd_info=True)
                    warehouse_option = self.accept_input(-1,table_info[0],"Select the warehouse to which the worker needs to be re-assigned to: ")
                    warehouse_id = df['warehouse_id'].values[warehouse_option]
                    update_statement = "update mover set warehouse_id="+str(warehouse_id)+" where mover_id="+str(mover_id)
                    self.cursor_obj.execute(update_statement)
                    print("Workplace succesfully updated")
                    time.sleep(1)
            else:
                break



    def get_data(self, statement, retrun_pd_info=False, show_table=True):
        temp_list = []
        self.cursor_obj.execute(statement)
        output = self.cursor_obj.fetchall()
        names = [description[0] for description in self.cursor_obj.description]
        df = pd.DataFrame(output, columns = names)
        if(show_table):
            self.show_df(df)
        if(retrun_pd_info):
            return df, [len(output), len(names)]

    def test(self):
        self.request_login()
        self.connection_obj.commit()
        self.connection_obj.close()





hello=dbms_application()



