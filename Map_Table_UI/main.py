import sys
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QComboBox, QLineEdit,
    QPushButton, QTableWidget, QTableWidgetItem, QLabel
)
from PyQt5.QtGui import QColor


# 生成状态位映射表，在编码模式下生成索引值
def statebits_map_table(maxValues):
    numFlags = len(maxValues)

    # 计算每个标志位的位宽和偏移量
    bitWidths = [int(np.ceil(np.log2(v + 1))) for v in maxValues]
    shifts = np.cumsum([0] + bitWidths[:-1])

    # 计算总组合数量
    totalCombinations = np.prod([v + 1 for v in maxValues])

    # 初始化状态表
    stateTable = np.zeros((totalCombinations, numFlags + 1), dtype=int)

    # 生成所有组合
    for combo in range(totalCombinations):
        # 解码组合号得到每个标志位的值
        flagValues = decode_combination(combo, maxValues)

        # 检查是否超出每个状态位的最大值范围
        if any(flagValues > maxValues):
            continue  # 跳过超出范围的组合

        # 计算索引值
        index = sum(flagValues[i] * (2 ** shifts[i]) for i in range(numFlags))

        # 填写表格
        stateTable[combo, :numFlags] = flagValues
        stateTable[combo, numFlags] = index  # 索引值

    # 删除未使用的表格行
    stateTable = stateTable[~np.all(stateTable == 0, axis=1)]

    return stateTable


def decode_combination(combo, maxValues):
    # 解码组合号得到每个标志位的值
    numFlags = len(maxValues)
    values = np.zeros(numFlags, dtype=int)

    for i in range(numFlags - 1, -1, -1):
        values[i] = combo % (maxValues[i] + 1)
        combo //= (maxValues[i] + 1)

    return values


# 主应用程序窗口类
class App(QWidget):
    def __init__(self):
        super().__init__()
        self.file_path = r"D:\STM32CubeMx_Projest\-R2\R2_F4_cpp\R2_USER\Xbox_Map_Table.h"  # 指定生成表格的.h文件路径
        self.initUI()

    def initUI(self):
        self.setWindowTitle('状态位映射表生成器')  # 设置窗口标题
        layout = QVBoxLayout()
        self.setLayout(layout)

        # 下拉菜单用于选择状态位数量
        self.maxValuesComboBox = QComboBox()
        self.maxValuesComboBox.addItems(['1', '2', '3', '4'])
        layout.addWidget(QLabel('选择状态位数量:'))
        layout.addWidget(self.maxValuesComboBox)
        self.maxValuesComboBox.currentTextChanged.connect(self.updateInputs)  # 更新输入框

        # 创建用于输入每个状态位最大值的文本框
        self.maxValueInputs = []
        self.createMaxValueInputs(1)  # 默认创建一个输入框

        # 按钮用于生成映射表
        self.generateButton = QPushButton('生成表格')
        self.generateButton.clicked.connect(self.generateTable)  # 点击生成表格
        layout.addWidget(self.generateButton)

        # 创建用于选择每个状态位值的下拉框
        self.stateValueDropdowns = []
        self.createDropdowns(1)  # 默认创建一个下拉框

        # 按钮用于高亮对应行
        self.highlightButton = QPushButton('高亮行')
        self.highlightButton.clicked.connect(self.highlightRow)  # 点击高亮行
        layout.addWidget(self.highlightButton)

        # 表格控件用于显示生成的状态映射表
        self.tableWidget = QTableWidget()
        layout.addWidget(self.tableWidget)

    # 根据选择的状态位数量创建最大值输入框
    def createMaxValueInputs(self, numFlags):
        # 清除之前的输入框
        for inputField in self.maxValueInputs:
            inputField.deleteLater()
        self.maxValueInputs = []

        # 重新创建输入框，并连接信号
        for i in range(numFlags):
            inputField = QLineEdit()
            inputField.setPlaceholderText(f"最大值{i + 1}")  # 设置占位符
            inputField.textChanged.connect(self.updateDropdownOptions)  # 连接到更新下拉选项
            self.maxValueInputs.append(inputField)
            self.layout().addWidget(inputField)

    # 创建选择状态值的下拉框
    def createDropdowns(self, numFlags):
        for dropdown in self.stateValueDropdowns:
            dropdown.deleteLater()
        self.stateValueDropdowns = []

        for i in range(numFlags):
            dropdown = QComboBox()
            self.stateValueDropdowns.append(dropdown)
            self.layout().addWidget(dropdown)

    # 更新输入框和下拉框
    def updateInputs(self):
        numFlags = int(self.maxValuesComboBox.currentText())
        self.createMaxValueInputs(numFlags)
        self.createDropdowns(numFlags)
        self.updateDropdownOptions()  # 更新下拉框选项

    # 更新下拉框的选项，根据最大值输入框的值动态生成
    def updateDropdownOptions(self):
        try:
            maxValues = [int(inputField.text()) for inputField in self.maxValueInputs]
            for i, dropdown in enumerate(self.stateValueDropdowns):
                dropdown.clear()  # 清空下拉框
                dropdown.addItems([str(x) for x in range(maxValues[i] + 1)])  # 添加选项
        except ValueError:
            pass  # 如果输入不是有效整数，保持下拉框为空

    def generateTable(self):
        numFlags = int(self.maxValuesComboBox.currentText())

        try:
            maxValues = [int(inputField.text()) for inputField in self.maxValueInputs]
            if len(maxValues) != numFlags or any(v < 0 for v in maxValues):
                print("请输入正确数量和非负值的最大值。")
                return
        except ValueError:
            print("请输入有效的最大值。")
            return

        self.stateTable = statebits_map_table(maxValues)

        if self.stateTable.size == 0:
            print("生成的状态表为空，无法写入文件。")
            return

        # 设置表格的行数和列数
        self.tableWidget.setRowCount(self.stateTable.shape[0])
        self.tableWidget.setColumnCount(numFlags + 1)
        self.tableWidget.setHorizontalHeaderLabels([f'状态位{i + 1}' for i in range(numFlags)] + ['索引'])

        for row, values in enumerate(self.stateTable):
            for col, value in enumerate(values):
                self.tableWidget.setItem(row, col, QTableWidgetItem(str(int(value))))
        self.writeToHeaderFile(maxValues)  # 调用写入文件的方法

    # 将数组写入指定的.h文件
    def writeToHeaderFile(self, maxValues):
        rows, cols = self.stateTable.shape  # 获取表格的行数和列数

        with open(self.file_path, 'w') as f:
            f.write("#ifndef STATE_BITS_H\n")
            f.write("#define STATE_BITS_H\n\n")
            f.write("#include <cstdint>\n\n")
            # 定义二维数组的行数和列数
            f.write(f"const int_fast16_t state_bits_array[{rows}][{cols}] = {{\n")

            # 填入每行数据
            for row in self.stateTable:
                row_str = ", ".join(map(str, row))  # 将每行转换为逗号分隔的字符串
                f.write(f"    {{{row_str}}},\n")  # 将行写入文件

            f.write("};\n\n")

            # 定义 max_values 数组
            f.write(f"const int_fast8_t max_values[{len(maxValues)}] = {{{', '.join(map(str, maxValues))}}};\n\n")
            f.write("#endif // STATE_BITS_H\n")


    # 根据用户选择的状态位值高亮对应行
    def highlightRow(self):
        try:
            userValues = [int(dropdown.currentText()) for dropdown in self.stateValueDropdowns]
            matchingRow = None

            for row, values in enumerate(self.stateTable):
                if np.array_equal(values[:-1], userValues):
                    matchingRow = row
                    break

            self.clearHighlights()

            if matchingRow is not None:
                for col in range(self.tableWidget.columnCount()):
                    item = self.tableWidget.item(matchingRow, col)
                    if item:
                        item.setBackground(QColor(255, 255, 0))  # 使用黄色高亮
                        item.setForeground(QColor(0, 0, 0))  # 设置字体颜色为黑色
                self.tableWidget.scrollToItem(self.tableWidget.item(matchingRow, 0))

        except ValueError:
            print("请输入有效的整数。")

    # 清除表格中所有的高亮
    def clearHighlights(self):
        for row in range(self.tableWidget.rowCount()):
            for col in range(self.tableWidget.columnCount()):
                item = self.tableWidget.item(row, col)
                if item and item.background().color() == QColor(255, 255, 0):
                    item.setForeground(QColor(0, 0, 0))  # 重新设置字体颜色为黑色

    # 关闭事件时清空表格
    def closeEvent(self, event):
        self.tableWidget.setRowCount(0)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    ex.show()
    sys.exit(app.exec_())
