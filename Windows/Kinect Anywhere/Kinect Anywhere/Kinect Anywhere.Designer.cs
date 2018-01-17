namespace Kinect_Anywhere
{
    partial class KinectAnywhereForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(KinectAnywhereForm));
            this.flowLayoutPanel1 = new System.Windows.Forms.FlowLayoutPanel();
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.dataLabel = new System.Windows.Forms.Label();
            this.colorDataLabel = new System.Windows.Forms.Label();
            this.bodyDataLabel = new System.Windows.Forms.Label();
            this.pointCloudDataLabel = new System.Windows.Forms.Label();
            this.valueLabel = new System.Windows.Forms.Label();
            this.ColorData = new System.Windows.Forms.CheckBox();
            this.BodyData = new System.Windows.Forms.CheckBox();
            this.PointCloudData = new System.Windows.Forms.CheckBox();
            this.label1 = new System.Windows.Forms.Label();
            this.startButton = new System.Windows.Forms.Button();
            this.trayIcon = new System.Windows.Forms.NotifyIcon(this.components);
            this.contextMenuStrip = new System.Windows.Forms.ContextMenuStrip(this.components);
            this.closeToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.flowLayoutPanel1.SuspendLayout();
            this.tableLayoutPanel1.SuspendLayout();
            this.contextMenuStrip.SuspendLayout();
            this.SuspendLayout();
            // 
            // flowLayoutPanel1
            // 
            this.flowLayoutPanel1.Controls.Add(this.tableLayoutPanel1);
            this.flowLayoutPanel1.Controls.Add(this.label1);
            this.flowLayoutPanel1.Controls.Add(this.startButton);
            this.flowLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flowLayoutPanel1.FlowDirection = System.Windows.Forms.FlowDirection.TopDown;
            this.flowLayoutPanel1.Location = new System.Drawing.Point(0, 0);
            this.flowLayoutPanel1.Name = "flowLayoutPanel1";
            this.flowLayoutPanel1.Size = new System.Drawing.Size(284, 261);
            this.flowLayoutPanel1.TabIndex = 0;
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.CellBorderStyle = System.Windows.Forms.TableLayoutPanelCellBorderStyle.Single;
            this.tableLayoutPanel1.ColumnCount = 2;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel1.Controls.Add(this.dataLabel, 0, 0);
            this.tableLayoutPanel1.Controls.Add(this.colorDataLabel, 0, 1);
            this.tableLayoutPanel1.Controls.Add(this.bodyDataLabel, 0, 2);
            this.tableLayoutPanel1.Controls.Add(this.pointCloudDataLabel, 0, 3);
            this.tableLayoutPanel1.Controls.Add(this.valueLabel, 1, 0);
            this.tableLayoutPanel1.Controls.Add(this.ColorData, 1, 1);
            this.tableLayoutPanel1.Controls.Add(this.BodyData, 1, 2);
            this.tableLayoutPanel1.Controls.Add(this.PointCloudData, 1, 3);
            this.tableLayoutPanel1.Location = new System.Drawing.Point(3, 10);
            this.tableLayoutPanel1.Margin = new System.Windows.Forms.Padding(3, 10, 3, 3);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 4;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 25F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 25F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 25F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 25F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(282, 136);
            this.tableLayoutPanel1.TabIndex = 0;
            // 
            // dataLabel
            // 
            this.dataLabel.AutoSize = true;
            this.dataLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.dataLabel.Location = new System.Drawing.Point(4, 1);
            this.dataLabel.Name = "dataLabel";
            this.dataLabel.Size = new System.Drawing.Size(33, 15);
            this.dataLabel.TabIndex = 0;
            this.dataLabel.Text = "Data";
            // 
            // colorDataLabel
            // 
            this.colorDataLabel.AutoSize = true;
            this.colorDataLabel.Location = new System.Drawing.Point(4, 34);
            this.colorDataLabel.Name = "colorDataLabel";
            this.colorDataLabel.Size = new System.Drawing.Size(57, 13);
            this.colorDataLabel.TabIndex = 1;
            this.colorDataLabel.Text = "Color Data";
            // 
            // bodyDataLabel
            // 
            this.bodyDataLabel.AutoSize = true;
            this.bodyDataLabel.Location = new System.Drawing.Point(4, 67);
            this.bodyDataLabel.Name = "bodyDataLabel";
            this.bodyDataLabel.Size = new System.Drawing.Size(57, 13);
            this.bodyDataLabel.TabIndex = 2;
            this.bodyDataLabel.Text = "Body Data";
            // 
            // pointCloudDataLabel
            // 
            this.pointCloudDataLabel.AutoSize = true;
            this.pointCloudDataLabel.Location = new System.Drawing.Point(4, 100);
            this.pointCloudDataLabel.Name = "pointCloudDataLabel";
            this.pointCloudDataLabel.Size = new System.Drawing.Size(87, 13);
            this.pointCloudDataLabel.TabIndex = 3;
            this.pointCloudDataLabel.Text = "Point Cloud Data";
            // 
            // valueLabel
            // 
            this.valueLabel.AutoSize = true;
            this.valueLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.valueLabel.Location = new System.Drawing.Point(144, 1);
            this.valueLabel.Name = "valueLabel";
            this.valueLabel.Size = new System.Drawing.Size(38, 15);
            this.valueLabel.TabIndex = 4;
            this.valueLabel.Text = "Value";
            // 
            // ColorData
            // 
            this.ColorData.AutoSize = true;
            this.ColorData.Location = new System.Drawing.Point(144, 37);
            this.ColorData.Name = "ColorData";
            this.ColorData.Size = new System.Drawing.Size(80, 17);
            this.ColorData.TabIndex = 5;
            this.ColorData.Text = "checkBox1";
            this.ColorData.UseVisualStyleBackColor = true;
            this.ColorData.CheckedChanged += new System.EventHandler(this.ColorData_CheckedChanged);
            // 
            // BodyData
            // 
            this.BodyData.AutoSize = true;
            this.BodyData.Location = new System.Drawing.Point(144, 70);
            this.BodyData.Name = "BodyData";
            this.BodyData.Size = new System.Drawing.Size(80, 17);
            this.BodyData.TabIndex = 6;
            this.BodyData.Text = "checkBox2";
            this.BodyData.UseVisualStyleBackColor = true;
            this.BodyData.CheckedChanged += new System.EventHandler(this.BodyData_CheckedChanged);
            // 
            // PointCloudData
            // 
            this.PointCloudData.AutoSize = true;
            this.PointCloudData.Location = new System.Drawing.Point(144, 103);
            this.PointCloudData.Name = "PointCloudData";
            this.PointCloudData.Size = new System.Drawing.Size(80, 17);
            this.PointCloudData.TabIndex = 7;
            this.PointCloudData.Text = "checkBox3";
            this.PointCloudData.UseVisualStyleBackColor = true;
            this.PointCloudData.CheckedChanged += new System.EventHandler(this.PointCloudData_CheckedChanged);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.label1.Location = new System.Drawing.Point(3, 159);
            this.label1.Margin = new System.Windows.Forms.Padding(3, 10, 3, 10);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(282, 13);
            this.label1.TabIndex = 1;
            this.label1.Text = "Tips: For better performance, disable unwanted data";
            // 
            // startButton
            // 
            this.startButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.startButton.Location = new System.Drawing.Point(3, 185);
            this.startButton.Name = "startButton";
            this.startButton.Size = new System.Drawing.Size(275, 50);
            this.startButton.TabIndex = 2;
            this.startButton.Text = "Start Kinect and minimize to System Tray";
            this.startButton.UseVisualStyleBackColor = true;
            this.startButton.Click += new System.EventHandler(this.StartButton_Click);
            // 
            // trayIcon
            // 
            this.trayIcon.ContextMenuStrip = this.contextMenuStrip;
            this.trayIcon.Icon = ((System.Drawing.Icon)(resources.GetObject("trayIcon.Icon")));
            this.trayIcon.Text = "Kinect Anywhere";
            this.trayIcon.Visible = true;
            // 
            // contextMenuStrip
            // 
            this.contextMenuStrip.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.closeToolStripMenuItem});
            this.contextMenuStrip.Name = "contextMenuStrip1";
            this.contextMenuStrip.Size = new System.Drawing.Size(104, 26);
            // 
            // closeToolStripMenuItem
            // 
            this.closeToolStripMenuItem.Name = "closeToolStripMenuItem";
            this.closeToolStripMenuItem.Size = new System.Drawing.Size(103, 22);
            this.closeToolStripMenuItem.Text = "Close";
            this.closeToolStripMenuItem.Click += new System.EventHandler(this.CloseToolStripMenuItem_Click);
            // 
            // KinectAnywhereForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(284, 261);
            this.Controls.Add(this.flowLayoutPanel1);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedSingle;
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.MaximizeBox = false;
            this.Name = "KinectAnywhereForm";
            this.Text = "Kinect Anywhere";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.KinectAnywhereForm_FormClosing);
            this.Load += new System.EventHandler(this.KinectAnywhereForm_Load);
            this.flowLayoutPanel1.ResumeLayout(false);
            this.flowLayoutPanel1.PerformLayout();
            this.tableLayoutPanel1.ResumeLayout(false);
            this.tableLayoutPanel1.PerformLayout();
            this.contextMenuStrip.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel1;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Button startButton;
        private System.Windows.Forms.Label dataLabel;
        private System.Windows.Forms.Label colorDataLabel;
        private System.Windows.Forms.Label bodyDataLabel;
        private System.Windows.Forms.Label pointCloudDataLabel;
        private System.Windows.Forms.Label valueLabel;
        private System.Windows.Forms.CheckBox ColorData;
        private System.Windows.Forms.CheckBox BodyData;
        private System.Windows.Forms.CheckBox PointCloudData;
        private System.Windows.Forms.NotifyIcon trayIcon;
        private System.Windows.Forms.ContextMenuStrip contextMenuStrip;
        private System.Windows.Forms.ToolStripMenuItem closeToolStripMenuItem;
    }
}

