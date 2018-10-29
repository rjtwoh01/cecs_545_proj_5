using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace TravelingSalesPerson
{
    /// <summary>
    /// Interaction logic for GeneticAlgorithmOptions.xaml
    /// </summary>
    public partial class GeneticAlgorithmOptions : Window
    {
        private MainWindow mainWindow;
        private bool forceClose = false;

        public GeneticAlgorithmOptions(MainWindow mainWindow)
        {
            this.mainWindow = mainWindow;
            InitializeComponent();
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (!this.forceClose)
            {
                this.Hide();
                e.Cancel = true;
            }
        }

        private void solveButton_Click(object sender, RoutedEventArgs e)
        {
            this.mainWindow.solveGeneticAlgorithm(Convert.ToInt32(crossOverTextBox.Text), Convert.ToInt32(mutationTextBox.Text), Convert.ToInt32(initialPopulationTextBox.Text), Convert.ToInt32(iterationsTextBox.Text), Convert.ToInt32(trialsTextBox.Text), Convert.ToBoolean(this.wocChecked.IsChecked));
            this.Hide();
        }

        public void ForceClose()
        {
            this.forceClose = true;
            this.Close();
        }
    }
}
