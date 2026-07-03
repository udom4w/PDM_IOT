import xml.etree.ElementTree as ET

# 读取 product-models.txt 文件
with open('product-models.txt', 'r', encoding='utf-8') as f:
    models = f.read().splitlines()

# 创建 XML 根元素
root = ET.Element('Products', xmlns_xsi='http://www.w3.org/2001/XMLSchema-instance', xmlns_xsd='http://www.w3.org/2001/XMLSchema')

for model in models:
    product_info = ET.SubElement(root, 'ProductInfo')
    ET.SubElement(product_info, 'ProductModelCN').text = model
    ET.SubElement(product_info, 'ProductModelEN').text = model
    ET.SubElement(product_info, 'Picture').text = f'JY901S.png'
    ET.SubElement(product_info, 'DataLinkCN').text = 'https://wit-motion.cn'
    ET.SubElement(product_info, 'DataLinkEN').text = 'https://wit-motion.com'
    ET.SubElement(product_info, 'CategoryCode').text = 'Attitude_Module'
    ET.SubElement(product_info, 'Powerby').text = model

# 创建 XML 树并写入文件
tree = ET.ElementTree(root)
ET.indent(tree)
tree.write('products.xml', encoding='utf-8', xml_declaration=True)