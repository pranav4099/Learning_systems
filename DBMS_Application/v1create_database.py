import sqlite3
import os

os.system("rm database.db")

connection_obj = sqlite3.connect('database.db')

cursor_obj = connection_obj.cursor()
cursor_obj.execute("PRAGMA foreign_keys;")
cursor_obj.execute("PRAGMA foreign_keys = 1;")

table_port =  """ 
create table port
(
port_id numeric (5,0),
address nvarchar (1000),
primary key (port_id)
); 
"""

table_port_authority = """
create table port_authority
(
port_authority_id numeric (5,0),
password nvarchar (1000),
port_id numeric (5,0),
primary key (port_authority_id),
foreign key (port_id) references port(port_id) on delete cascade
);
"""

table_manufacturer = """
create table manufacturer
(
seller_id numeric (5,0), 
password nvarchar (1000),
primary key (seller_id)
);
"""

table_cargo = """
create table cargo
(
    cargo_id numeric (5,0),
    seller_id numeric (5,0), 
    destination nvarchar (1000),
    warehouse_id nvarchar (1000),
    port_authority_id numeric (5,0),
    port_id numeric (5,0),
    safety_check bool,
    primary key (cargo_id, seller_id),
    foreign key (port_authority_id) references port_authority(port_authority_id) on delete cascade,
    foreign key (port_id) references port(port_id) on delete cascade,
    foreign key (seller_id) references manufacturer(seller_id) on delete cascade
);
"""


table_retailer = """
create table retailer
(
buyer_id numeric (5,0),
password nvarchar (1000),
total_orders_placed int,
primary key (buyer_id)
);
"""

table_wholesaler ="""
create table wholesaler
(
buyer_id numeric (5,0),
password nvarchar (1000),
total_orders_placed int,
primary key (buyer_id)
);
"""

table_purchase_retailer ="""
create table purchase_retailer
(
buyer_id numeric (5,0),
catalog_id numeric (5,0),
seller_id numeric (5,0),
cargo_id numeric (5,0),
size int,
cancel bool,
primary key (seller_id, buyer_id, cargo_id),
foreign key (buyer_id) references retailer(buyer_id) on delete cascade,
foreign key (seller_id, cargo_id) references cargo(seller_id, cargo_id) on delete cascade
);
"""

table_purchase_wholesaler ="""
create table purchase_wholesaler
(
buyer_id numeric (5,0),
catalog_id numeric (5,0),
seller_id numeric (5,0),
cargo_id numeric (5,0),
size int,
cancel bool,
primary key (seller_id, buyer_id, cargo_id),
foreign key (buyer_id) references wholesaler(buyer_id) on delete cascade,
foreign key (seller_id, cargo_id) references cargo(seller_id, cargo_id) on delete cascade
);
"""

table_warehouse ="""
create table warehouse
(
warehouse_id numeric (5,0),
warehouse_address nvarchar (1000),
primary key (warehouse_id)
);
"""

table_holds ="""
create table holds
(
warehouse_id numeric (5,0),
seller_id numeric (5,0),
cargo_id numeric (5,0),
primary key (warehouse_id, seller_id, cargo_id),
foreign key (warehouse_id) references warehouse(warehouse_id) on delete cascade,
foreign key (seller_id, cargo_id) references cargo(seller_id, cargo_id) on delete cascade
);
"""

table_manager ="""
create table manager
(
worker_id numeric (5,0),
password nvarchar (1000),
primary key (worker_id)
);
"""

table_mover ="""
create table mover
(
worker_id numeric (5,0),
password nvarchar (1000),
warehouse_id numeric (5,0),
manager_id numeric (5,0),
primary key (worker_id),
foreign key (warehouse_id) references warehouse(warehouse_id) on delete cascade,
foreign key (manager_id) references manager(worker_id) on delete cascade
);
"""

table_insurance_company ="""
create table insurance_company
(
company_id numeric (5,0),
listing_price int,
password nvarchar (1000),
primary key (company_id)
);
"""


table_insurance_plan ="""
create table insurance_purchase
(
company_id numeric (5,0),
buyer_id numeric (5,0),
foreign key (company_id) references insurance_company(company_id) on delete cascade
);
"""

table_catalog ="""
create table catalog
(
catalog_id numeric (5,0),
seller_id numeric (5,0),
item_name nvarchar (1000),
item_price int,
primary key (catalog_id, seller_id),
foreign key (seller_id) references manufacturer(seller_id) on delete cascade
);
"""

table_dba ="""
create table dba
(
dba_id numeric (5,0),
password nvarchar (1000),
primary key (dba_id)
);
"""

cursor_obj.execute(table_dba)
cursor_obj.execute(table_port)
cursor_obj.execute(table_port_authority)
cursor_obj.execute(table_manufacturer)
cursor_obj.execute(table_cargo)
cursor_obj.execute(table_retailer)
cursor_obj.execute(table_wholesaler)
cursor_obj.execute(table_purchase_retailer)
cursor_obj.execute(table_purchase_wholesaler)
cursor_obj.execute(table_warehouse)
cursor_obj.execute(table_holds)
cursor_obj.execute(table_manager)
cursor_obj.execute(table_mover)
cursor_obj.execute(table_insurance_company)
cursor_obj.execute(table_insurance_plan)
cursor_obj.execute(table_catalog)

cursor_obj.execute('''INSERT INTO  dba VALUES(1, 'a')''')

cursor_obj.execute('''INSERT INTO  port VALUES(1, 'LA')''')
cursor_obj.execute('''INSERT INTO  port VALUES(2, 'Florida')''')

cursor_obj.execute('''INSERT INTO  warehouse VALUES(1, 'Boston')''')
cursor_obj.execute('''INSERT INTO  warehouse VALUES(2, 'Chicago')''')

cursor_obj.execute('''INSERT INTO  manager VALUES(1, 'a')''')
cursor_obj.execute('''INSERT INTO  manager VALUES(2, 'a')''')

cursor_obj.execute('''INSERT INTO  mover VALUES(1, 'a', 1, 1)''')
cursor_obj.execute('''INSERT INTO  mover VALUES(2, 'a', 2, 2)''')

cursor_obj.execute('''INSERT INTO  manufacturer VALUES(1, 'a')''')

cursor_obj.execute('''INSERT INTO  port_authority VALUES(1, 'a', 1)''')
cursor_obj.execute('''INSERT INTO  port_authority VALUES(2, 'a', 2)''')

cursor_obj.execute('''INSERT INTO catalog VALUES(1, 1, 'Batman', 2000)''')
cursor_obj.execute('''INSERT INTO catalog VALUES(2, 1, 'Superman', 750)''')

cursor_obj.execute('''INSERT INTO retailer VALUES(1, 'a', 2)''')
cursor_obj.execute('''INSERT INTO wholesaler VALUES(2, 'a', 2)''')

cursor_obj.execute('''INSERT INTO insurance_company VALUES(1, 599, 'a')''')
cursor_obj.execute('''INSERT INTO insurance_company VALUES(2, 799, 'a')''')

print("Database is Ready")

connection_obj.commit()
connection_obj.close()
